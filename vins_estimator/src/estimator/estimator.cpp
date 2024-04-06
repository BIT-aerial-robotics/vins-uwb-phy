/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"

Estimator::Estimator() : f_manager{Rs}
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    clearState();
    
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }
}

void Estimator::clearState()
{
    mProcess.lock();
    to_world_rt_flag = 0;
    while (!accBuf.empty())
        accBuf.pop();
    while (!gyrBuf.empty())
        gyrBuf.pop();
    while (!featureBuf.empty())
        featureBuf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

void Estimator::setParameter()
{
    mProcess.lock();
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl
             << ric[i] << endl
             << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES);
    line_feature_tracker.readIntrinsicParameter(CAM_NAMES);
    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    para_UWB_anchor[0][0] = -38.17;
    para_UWB_anchor[0][1] = -34.35;
    para_UWB_anchor[0][2] = 1.38;

    para_UWB_anchor[1][0] = 32.93;
    para_UWB_anchor[1][1] = -36.65;
    para_UWB_anchor[1][2] = 3.3;

    para_UWB_anchor[2][0] = 38.76;
    para_UWB_anchor[2][1] = 46.12;
    para_UWB_anchor[2][2] = 1.59;

    para_UWB_anchor[3][0] = -34.48;
    para_UWB_anchor[3][1] = 31.17;
    para_UWB_anchor[3][2] = 1.14;

    para_UWB_anchor[4][0] = 5;
    para_UWB_anchor[4][1] = 4;
    para_UWB_anchor[4][2] = 3.0;

    for (int uwbIdx = 0; uwbIdx <= 4; uwbIdx++)
    {
        if(SIM_UE)
        {
            //para_UWB_anchor[uwbIdx][0]*=2;
            //para_UWB_anchor[uwbIdx][1]*=2;
            //para_UWB_anchor[uwbIdx][2]*=2;
        }
    }
    for (int uwbIdx = 0; uwbIdx <= 4; uwbIdx++)
    {
        para_UWB_bias[uwbIdx][0] = 0.0;
        UWB_anchor[uwbIdx] = Eigen::Vector3d(para_UWB_anchor[uwbIdx][0], para_UWB_anchor[uwbIdx][1], para_UWB_anchor[uwbIdx][2]);
    }
    

    if(SIM_UE==1){
        para_tag[0]=para_tag[1]=para_tag[2]=0;
        para_length[0]=0.957;
        para_hinge[0]=0;
        para_hinge[1]=0;
        para_hinge[2]=0;
    }
    else{
        para_hinge[0]=HINGE(0);
        para_hinge[1]=HINGE(1);
        para_hinge[2]=HINGE(2);

        para_tag[0]=HINGE(0);
        para_tag[1]=HINGE(1);
        para_tag[2]=0.03;
        para_length[0]=0.841;
    }
    para_self_len[0]=0.0;
    mProcess.unlock();
}

void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if (!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if (USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if (USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }

        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if (restart)
    {
        clearState();
        setParameter();
    }
}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    map<int, vector<pair<int, Eigen::Vector4d>>> line;
    TicToc featureTrackerTime;

    if (_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    // printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (USELINE)
    {

        if (_img1.empty())
            line = line_feature_tracker.trackImage(t, _img);
        else
            line = line_feature_tracker.trackImage(t, _img, _img1);
    }
    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        if (USELINE)
        {
            cv::Mat imgTrackLine = line_feature_tracker.getTrackImage();
            pubTrackImageLine(imgTrackLine, t);
        }

        pubTrackImage(imgTrack, t);
    }

    if (MULTIPLE_THREAD)
    {
        if (inputImageCnt % 2 == 0)
        {
            mBuf.lock();
            if (USELINE)
                featureWithLineBuf.push(make_pair(t, make_pair(featureFrame, line)));
            else
                featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        if (USELINE)
            featureWithLineBuf.push(make_pair(t, make_pair(featureFrame, line)));
        else
            featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    // printf("input imu with time %f \n", t);
    mBuf.unlock();

    if (solver_flag == NON_LINEAR)
    {
        mPropagate.lock();
        fastPredictIMU(t, linearAcceleration, angularVelocity);
        double range[10];
        for(int i=0;i<=3;i++){
            getRange(i,t,range[i]);
        }
        // OdometryVins tmp;
        // OdometryVins::queryOdometryMap(gt_map[AGENT_NUMBER],t,tmp,0.1);
        if (to_world_rt_flag)
        {
            pubLatestOdometry(latest_P, latest_Q, latest_V,angularVelocity-latest_Bg,t, mat_2_world);
        }
        
        pubLatestOdometry(latest_P, latest_Q, latest_V, t,range);
        mPropagate.unlock();
    }
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    mBuf.lock();
    featureBuf.push(make_pair(t, featureFrame));
    mBuf.unlock();

    if (!MULTIPLE_THREAD)
        processMeasurements();
}

bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                               vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if (accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    // printf("get imu from %f %f\n", t0, t1);
    // printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if (t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    if (!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

void Estimator::processMeasurements()
{
    while (1)
    {
        // printf("process measurments\n");
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if (USELINE == 0)
        {
            if (!featureBuf.empty())
            {
                feature = featureBuf.front();
                curTime = feature.first + td;
                while (1)
                {
                    if ((!USE_IMU || IMUAvailable(feature.first + td)))
                        break;
                    else
                    {
                        printf("wait for imu ... \n");
                        if (!MULTIPLE_THREAD)
                            return;
                        std::chrono::milliseconds dura(5);
                        std::this_thread::sleep_for(dura);
                    }
                }
                mBuf.lock();
                if (USE_IMU)
                    getIMUInterval(prevTime, curTime, accVector, gyrVector);

                featureBuf.pop();
                mBuf.unlock();

                if (USE_IMU)
                {
                    if (!initFirstPoseFlag)
                        initFirstIMUPose(accVector);
                    for (size_t i = 0; i < accVector.size(); i++)
                    {
                        double dt;
                        if (i == 0)
                            dt = accVector[i].first - prevTime;
                        else if (i == accVector.size() - 1)
                            dt = curTime - accVector[i - 1].first;
                        else
                            dt = accVector[i].first - accVector[i - 1].first;
                        processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                    }
                }
                mProcess.lock();
                processImage(feature.second, feature.first);
                prevTime = curTime;

                printStatistics(*this, 0);

                std_msgs::Header header;
                header.frame_id = "world";
                header.stamp = ros::Time(feature.first);

                pubOdometry(*this, header);
                pubKeyPoses(*this, header);
                pubCameraPose(*this, header);
                pubPointCloud(*this, header);
                pubKeyframe(*this);
                pubTF(*this, header);
                mProcess.unlock();
            }
        }
        else
        {
            if (!featureWithLineBuf.empty())
            {
                pair<double, pair<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>,
                                  map<int, vector<pair<int, Vector4d>>>>>
                    feature_with_line;
                feature_with_line = featureWithLineBuf.front();
                curTime = feature_with_line.first + td;
                while (1)
                {
                    if ((!USE_IMU || IMUAvailable(feature_with_line.first + td)))
                        break;
                    else
                    {
                        printf("wait for imu ... \n");
                        if (!MULTIPLE_THREAD)
                            return;
                        std::chrono::milliseconds dura(5);
                        std::this_thread::sleep_for(dura);
                    }
                }
                mBuf.lock();
                if (USE_IMU)
                    getIMUInterval(prevTime, curTime, accVector, gyrVector);

                featureWithLineBuf.pop();
                mBuf.unlock();

                if (USE_IMU)
                {
                    if (!initFirstPoseFlag)
                        initFirstIMUPose(accVector);
                    for (size_t i = 0; i < accVector.size(); i++)
                    {
                        double dt;
                        if (i == 0)
                            dt = accVector[i].first - prevTime;
                        else if (i == accVector.size() - 1)
                            dt = curTime - accVector[i - 1].first;
                        else
                            dt = accVector[i].first - accVector[i - 1].first;
                        processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                    }
                }
                mProcess.lock();
                processImage(feature_with_line.second.first, feature_with_line.second.second, feature_with_line.first);
                prevTime = curTime;

                printStatistics(*this, 0);

                std_msgs::Header header;
                header.frame_id = "world";
                header.stamp = ros::Time(feature.first);

                pubOdometry(*this, header);
                pubKeyPoses(*this, header);
                pubCameraPose(*this, header);
                pubPointCloud(*this, header);
                pubKeyframe(*this);
                pubTF(*this, header);
                mProcess.unlock();
            }
        }

        if (!MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    // return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for (size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl
         << Rs[0] << endl;
    // Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}

void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity, t);
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity, t);
        // pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //  if(solver_flag != NON_LINEAR)
        // tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;
        // printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        // printf("non-keyframe\n");
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl
                                                               << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;
                }
                if (result)
                {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if (STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs);
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // stereo only initilization
        if (STEREO && !USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if (frame_count == WINDOW_SIZE)
            {
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        if (frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }
    }
    else
    {
        TicToc t_solve;
        if (!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        optimization();
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        if (!MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }

        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
        clearMap();
    }
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    // check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            // cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        // ROS_WARN("IMU variation %f!", var);
        if (var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            // return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if (!sfm.construct(frame_count + 1, Q, T, l,
                       relative_R, relative_T,
                       sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    // solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }
}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    // solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if (frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    // Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if (USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;

    if (USELINE)
    {
#ifdef LINEINCAM
        Eigen::MatrixXd lineorth = f_manager.getLineOrthVectorInCamera();
#else
        Eigen::MatrixXd lineorth = f_manager.getLineOrthVector(Ps, tic, ric);
#endif

        for (int i = 0; i < f_manager.getLineFeatureCount(); ++i)
        {
            para_LineFeature[i][0] = lineorth.row(i)[0];
            para_LineFeature[i][1] = lineorth.row(i)[1];
            para_LineFeature[i][2] = lineorth.row(i)[2];
            para_LineFeature[i][3] = lineorth.row(i)[3];
            if (i > NUM_OF_F)
                std::cerr << " 1000  1000 1000 1000 1000 \n\n";
        }
    }


    if(USE_LONG_WINDOW)
    {
        for (unsigned i = 0; i < Ps_long.size(); i++)
        {
            para_Pose_Long[i][0] = Ps_long.at(i).Ps.x();
            para_Pose_Long[i][1] = Ps_long.at(i).Ps.y();
            para_Pose_Long[i][2] = Ps_long.at(i).Ps.z();
            Quaterniond q=Ps_long.at(i).Rs;
            para_Pose_Long[i][3] = q.x();
            para_Pose_Long[i][4] = q.y();
            para_Pose_Long[i][5] = q.z();
            para_Pose_Long[i][6] = q.w();   
            //printf("%lf %lf %lf %lf %lf %lf %lf | ",para_Pose_Long[i][0],para_Pose_Long[i][1],para_Pose_Long[i][2],para_Pose_Long[i][3],para_Pose_Long[i][4],para_Pose_Long[i][5],para_Pose_Long[i][6]); 
        }//printf("\n");
    }
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if (Ps_long.size() > 0 && solver_flag == NON_LINEAR&&USE_LONG_WINDOW)
    {
        //RsLong_result.clear();
        // Ps_long_res.clear();
        // for (unsigned i = 0; i < Ps_long.size(); i++)
        // {
        //     //RsLong_result.push_back(rot_diff * Quaterniond(para_Pose_Long[i][6], para_Pose_Long[i][3], para_Pose_Long[i][4], para_Pose_Long[i][5]).normalized().toRotationMatrix());
            
        //     Ps_long_res.push_back( Vector3d(para_Pose_Long[i][0],
        //                     para_Pose_Long[i][1],
        //                     para_Pose_Long[i][2]));   


        // }

        Eigen::Vector3d long_window_lastone=Eigen::Vector3d(Vector3d(para_Pose_Long[Ps_long.size()-1]));

        if ((long_window_lastone-origin_P0).norm()<0.9 && (long_window_lastone-origin_P0).norm()>0.01)
        {
            //cout<<" ========== pose jump  ================ "<< endl;
            origin_P0 = long_window_lastone+Ps[0]-Ps_long.back().Ps;
            //Ps_long.clear();
            for (unsigned i = 0; i <Ps_long.size(); i++)
            {
                //RsLong_result.push_back(rot_diff * Quaterniond(para_Pose_Long[i][6], para_Pose_Long[i][3], para_Pose_Long[i][4], para_Pose_Long[i][5]).normalized().toRotationMatrix());
                
                Ps_long.at(i).Ps= Vector3d(para_Pose_Long[i][0],
                                para_Pose_Long[i][1],
                                para_Pose_Long[i][2]);   

            }
        }

        

        // virPos = Ps[WINDOW_SIZE]-Ps[0] + PsLong_result.back();
        // virOri = Rs[WINDOW_SIZE];
        // virVel = Vs[WINDOW_SIZE];



    }
    
    if (USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                         para_Pose[0][3],
                                                         para_Pose[0][4],
                                                         para_Pose[0][5])
                                                 .toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        // TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5])
                                   .toRotationMatrix()
                                   .transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            if(USE_UWB)
            {
                Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
                Ps[i] = Vector3d(para_Pose[i][0],
                                            para_Pose[i][1],
                                            para_Pose[i][2]);
                Vs[i] = Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);
            }
            else{
                Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                        para_Pose[i][1] - para_Pose[0][1],
                                        para_Pose[i][2] - para_Pose[0][2]) +origin_P0;
                Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);
                
            }
            Bas[i] = Vector3d(para_SpeedBias[i][3],
                              para_SpeedBias[i][4],
                              para_SpeedBias[i][5]);

            Bgs[i] = Vector3d(para_SpeedBias[i][6],
                              para_SpeedBias[i][7],
                              para_SpeedBias[i][8]);
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if (USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5])
                         .normalized()
                         .toRotationMatrix();
        }
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (USE_IMU)
        td = para_Td[0][0];

    if (USELINE)
    {
        // std::cout <<"----------\n"<< Rwow1 <<"\n"<<twow1<<std::endl;
        MatrixXd lineorth_vec(f_manager.getLineFeatureCount(), 4);
        for (int i = 0; i < f_manager.getLineFeatureCount(); ++i)
        {
            Vector4d orth(para_LineFeature[i][0],
                          para_LineFeature[i][1],
                          para_LineFeature[i][2],
                          para_LineFeature[i][3]);
            lineorth_vec.row(i) = orth;
        }
#ifdef LINEINCAM
        f_manager.setLineOrthInCamera(lineorth_vec);
#else
        f_manager.setLineOrth(lineorth_vec, Ps, Rs, tic, ric);
#endif
    }
}

bool Estimator::failureDetection()
{
    return false;
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        // return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        // ROS_INFO(" big translation");
        // return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        // ROS_INFO(" big z translation");
        // return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        // return true;
    }
    return false;
}

void Estimator::optimization()
{
    frame_sol_cnt++;
    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if (!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            // ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if (USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if (STEREO && it_per_frame.is_stereo)
            {
                Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
            }
            f_m_cnt++;
        }
    }

    // ROS_INFO("USE_UWB ==== %d \n",USE_UWB);

    if(USE_LONG_WINDOW&&Ps_long.size()>0)
    {
        for (int i = 0; i < Ps_long.size(); i++)
        {
            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_Pose_Long[i], SIZE_POSE, local_parameterization);
        }
        problem.SetParameterBlockConstant(para_Pose_Long[0]);
        for (int i = 0; i < Ps_long.size(); i++)
        {
            for (int j = 1; j < 5; j++)
            {
                int neibLink = i-j;
                if (neibLink >0)
                {
                    //cout<<" Add residual in pslong "<< i << " "<< neibLink << " pslong size "<<  PsLong.size() << " "<< endl;
                    ceres::CostFunction* cost_function = LongWindowError::Create(Ps_long.at(neibLink).Ps, Ps_long.at(i).Ps, Ps_long.at(neibLink).Rs.toRotationMatrix(), Ps_long.at(i).Rs.toRotationMatrix(), LINK_W);
                    problem.AddResidualBlock(cost_function, NULL, para_Pose_Long[neibLink], para_Pose_Long[i]);
                }
            }

            ceres::CostFunction* cost_function = movingError::Create(Ps_long.at(i).Ps,  MOVE_W);
            problem.AddResidualBlock(cost_function, NULL, para_Pose_Long[i]);

        }

        // //Link between pose in window and long window

        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            for (int j = 1; j <10&&j<LONG_WINDOW_SIZE; j++)
            {   
                unsigned neibLink = Ps_long.size() + i-j;
                if (neibLink<Ps_long.size())
                {
                    ceres::CostFunction* cost_function = LongWindowError::Create( Ps_long.at(neibLink).Ps,Ps[i], Ps_long.at(neibLink).Rs.toRotationMatrix(), Rs[i], 1);
                    problem.AddResidualBlock(cost_function, NULL, para_Pose_Long[neibLink], para_Pose[i]);
                }
            }
        }

        if(USE_UWB&&to_world_rt_flag)
        {
            Eigen::Vector3d sp1=mat_2_world.Ps;
            Eigen::Quaterniond sr1=mat_2_world.Rs;

            for(int i=0;i<Ps_long.size();i++){
                //printf("%lf %lf %lf %lf | ",Ps_long.at(i).range[0],Ps_long.at(i).range[1],Ps_long.at(i).range[2],Ps_long.at(i).range[3]);
                for(int uwbIdx=lowNum;uwbIdx<=uwbNum;uwbIdx++)
                {
                    if(Ps_long.at(i).range[uwbIdx]<0)continue;
                    UwbFactor *conxt = new UwbFactor(sp1, sr1.toRotationMatrix(), Ps_long.at(i).range[uwbIdx],0.15);
                    // problem.AddResidualBlock(
                    //     new ceres::AutoDiffCostFunction<UwbFactor, 1, 7, 3, 1,3>(conxt),
                    //     NULL,
                    //     para_Pose_Long[i], para_UWB_anchor[uwbIdx], para_UWB_bias[uwbIdx],para_tag);
                }
                
            }printf("\n");
        }
    }
    if (USE_UWB)
    {
        problem.AddParameterBlock(para_tag,3);
        problem.SetParameterBlockConstant(para_tag);
        for (int uwbIdx = 0; uwbIdx <= 3; uwbIdx++)
        {
            problem.AddParameterBlock(para_UWB_anchor[uwbIdx], 3);
            problem.AddParameterBlock(para_UWB_bias[uwbIdx], 1);
            // UwbFactor_old *conxt =new UwbFactor_old(para_UWB_Anchor[uwbIdx], 0.001);

            // problem.AddResidualBlock(
            //     new ceres::AutoDiffCostFunction<UwbFactor_old, 3, 3>(conxt),
            //         NULL,para_UWB_Anchor[uwbIdx]);

            problem.SetParameterBlockConstant(para_UWB_anchor[uwbIdx]);
            problem.SetParameterBlockConstant(para_UWB_bias[uwbIdx]);
            // UWBBiasFactor *conxt = new UWBBiasFactor(para_UWB_bias[uwbIdx][0], 0.008);
            // problem.AddResidualBlock(
            //     new ceres::AutoDiffCostFunction<UWBBiasFactor, 1, 1>(conxt),
            //     NULL, para_UWB_bias[uwbIdx]);
        }
        if (frame_sol_cnt >= 5)
        {
            uwb_length = 0;
            uwb_2_index.clear();
            // ROS_INFO("begin uwb add get Pose and UWB");
            getPoseAndUWB(uwb_length, uwb_2_index);

            int resNum = 0;
            int lostNum = 0;
            int safeMen = 0;

            if (uwb_length > 0)
            {
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    double time = Headers[i];
                    auto iter = uwb_2_index.lower_bound(time);
                    if (iter != uwb_2_index.end() && abs(time - iter->first) <= 0.02)
                    {
                        int nxt = iter->second;
                        for (int uwbIdx = lowNum; uwbIdx <= uwbNum; uwbIdx++)
                        {
                            //printf("distance %lf\n",uwb_mea[uwbIdx][nxt]);
                            if (uwb_can[uwbIdx][nxt])
                            {
                                Eigen::Vector3d sp1=mat_2_world.Ps, sp2;
                                Eigen::Quaterniond sr1=mat_2_world.Rs, sr2;
                                arrayTeigenYaw(para_uwb_local_world_Rt[nxt], sp1, sr1);
                                //sqrt(uwb_mea_sta[uwbIdx].variance())*1.5+(uwb_mea[uwbIdx][nxt]-uwb_mea_sta[uwbIdx].mean())*0.5
                                UwbFactor *conxt = new UwbFactor(sp1, sr1.toRotationMatrix(), uwb_mea[uwbIdx][nxt],0.02);
                                //printf("info from uwb %lf\n",sqrt(uwb_mea_sta[uwbIdx].variance())*2.5+(uwb_mea[uwbIdx][nxt]-uwb_mea_sta[uwbIdx].mean())*0.5);
                                problem.AddResidualBlock(
                                    new ceres::AutoDiffCostFunction<UwbFactor, 1, 7, 3, 1,3>(conxt),
                                    NULL,
                                    para_Pose[i], para_UWB_anchor[uwbIdx], para_UWB_bias[uwbIdx],para_tag);

                                //Eigen::Vector3d x(para_Pose[i]);
                                //x=sr1.toRotationMatrix()*x+sp1;
                                //Eigen::Vector3d y=x-UWB_anchor[uwbIdx];
                                //ROS_INFO("%d %d %lf %lf %lf %lf %lf error=%lf",uwbIdx,i,x.x(),x.y(),x.z(),uwb_mea[uwbIdx][nxt],y.norm(),y.norm()-uwb_mea[uwbIdx][nxt]);
                                //resNum += 1;
                            }
                        }
                    }
                    int cnt = pre_integrations[i]->last_t_buf.size();
                    int imu_frame_num; // min(cnt, imu_delta_fre);
                    int shift_num;     //= cnt / imu_frame_num;
                    if (cnt == 0 || pre_integrations[i]->sum_dt > 3 || cnt > 300 || IMU_SAEM_FRE == 0)
                    {
                        imu_frame_num = 0;
                        shift_num = 100;
                    }
                    else
                    {
                        imu_frame_num = min(cnt, imu_delta_fre);
                        shift_num = cnt / imu_frame_num;
                    }
                    int tt = 0;
                    for (int j = 1; j < cnt && i >= 1; j += shift_num)
                    {
                        time = pre_integrations[i]->last_t_buf[j];
                        auto iter = uwb_2_index.lower_bound(time);
                        if (iter == uwb_2_index.end() || abs(time - iter->first) > 0.02)
                            continue;
                        int nxt = iter->second;
                        Eigen::Vector3d delta_p = pre_integrations[i]->delta_p_buf[j];
                        Eigen::Vector3d delta_v = pre_integrations[i]->delta_v_buf[j];
                        Eigen::Matrix3d delta_q = pre_integrations[i]->delta_q_buf[j].toRotationMatrix();
                        Eigen::Vector3d xworldP, yworldP, eworldP=mat_2_world.Ps;
                        Eigen::Quaterniond xworldR, yworldR, eworldR=mat_2_world.Rs;
                        //arrayTeigenYaw(para_uwb_local_world_Rt[nxt], eworldP, eworldR);
                        for (int uwbIdx = lowNum; uwbIdx <= uwbNum; uwbIdx++)
                        {
                            if (uwb_can[uwbIdx][nxt])
                            {
                                //sqrt(uwb_mea_sta[uwbIdx].variance())*1.5+(uwb_mea[uwbIdx][nxt]-uwb_mea_sta[uwbIdx].mean())*0.5
                                UWBFactor_delta *conxt = new UWBFactor_delta(eworldP, eworldR.toRotationMatrix(),
                                                                             delta_p, delta_q, uwb_fre_time[nxt] - Headers[i - 1], uwb_mea[uwbIdx][nxt],0.02);
                                problem.AddResidualBlock(
                                    new ceres::AutoDiffCostFunction<UWBFactor_delta, 1, 7, 9, 3, 1,3>(conxt),
                                    NULL,
                                    para_Pose[i - 1], para_SpeedBias[i - 1], para_UWB_anchor[uwbIdx], para_UWB_bias[uwbIdx],para_tag);
                                resNum += 1;
                            }
                        }
                    }
                }
            }
            cout << "uwb_length:" << uwb_length << "  " << uwb_2_index.size() << "  " << resNum << endl;
        }
   
        double time = Headers[WINDOW_SIZE];
        auto iter = uwb_2_index.lower_bound(time);
        int flag_fir=0;
        if (iter != uwb_2_index.end() && abs(time - iter->first) <= 0.02)
        {
            int nxt = iter->second;
            int num=0;
            double alldis=0;
            double base=0;
            for (int uwbIdx = lowNum; uwbIdx <= uwbNum; uwbIdx++)
            {

                if (uwb_can[uwbIdx][nxt])
                {
                    Eigen::Vector3d sp1=mat_2_world.Ps, sp2(para_UWB_anchor[uwbIdx]),sp3=Ps[0],dis_vec;
                    Eigen::Quaterniond sr1=mat_2_world.Rs, sr3{Rs[0]};
                    sp3+=sr3*Eigen::Vector3d(para_tag);
                    sp3=sr1*sp3+sp1;
                    dis_vec=sp3-sp2;
                    double dis=dis_vec.norm()-(uwb_mea[uwbIdx][nxt]-para_UWB_bias[uwbIdx][0]);
                    alldis+=abs(dis);
                    num+=1;
                    base+=sqrt(uwb_mea_sta[uwbIdx].variance());
                }
            }

            if(num>0){
                // if(frame_sol_cnt%40==0&&frame_sol_cnt>40){
                //     sta.clear();
                // }
                // alldis/=num;
                // sta.update(alldis);
                // alldis=sta.mean();
                // base/=num;
                // double expbase=exp(1+base);
                // printf("base ==  %lf expbase %lf alldis == %lf",base,expbase,alldis);
                // kinFactor_old *old_fir = new kinFactor_old(para_Pose[0], 0.0005+(alldis/(40*expbase)),0.0001+(alldis/(40*expbase)));
                // problem.AddResidualBlock(
                //         new ceres::AutoDiffCostFunction<kinFactor_old, 7, 7>(old_fir),
                //         NULL,
                //         para_Pose[0]);
                // flag_fir=1;
            }
            
        }
        if(flag_fir==0){
            double base=0.001;
            if(AGENT_NUMBER!=2)base/=16;
            kinFactor_old *old_fir = new kinFactor_old(para_Pose[0], base*(USE_UWB+USE_KIN*2),base*0.5*(USE_UWB+USE_KIN*2));
            problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<kinFactor_old, 7, 7>(old_fir),
            NULL,
            para_Pose[0]);
        }
    }
    if (USE_KIN)
    {
        problem.AddParameterBlock(para_hinge,3);
        problem.AddParameterBlock(para_length,1);
        problem.SetParameterBlockConstant(para_hinge);
        problem.SetParameterBlockConstant(para_length);
        if (frame_sol_cnt >= 5)
        {
            
            kin_length = 0;
            kin_2_index.clear();
            if(USE_UWB){
                if(to_world_rt_flag)
                    getPoseAndOtherAgent(kin_length, kin_2_index);
            }
            else
                getPoseAndOtherAgent(kin_length, kin_2_index);
            
            int resNum = 0;
            int x, y;
            getIndexByCycle(x, y);
            if(kin_length>0){
                if(USE_EXR==0||USE_EXR==2)
                {
                    for(int i=1;i<=3;i++){
                        for(int j=0;j<kin_length;j++){
                            problem.AddParameterBlock(para_kin_local_world_Rt[i][j],4);
                            problem.SetParameterBlockConstant(para_kin_local_world_Rt[i][j]);
                        }
                    }
                }
                if(USE_EXR==1)
                {
                    for(int i=1;i<=3;i++){
                        if(i==AGENT_NUMBER)continue;
                        for(int j=0;j<kin_length;j++){
                            problem.AddParameterBlock(para_kin_local_world_Rt[i][j],4);
                            kinFactor_bet_old_4dof *old_ctx = new kinFactor_bet_old_4dof(
                                para_kin_local_world_Rt[i][j],sigma_vins_6dof
                            );
                            problem.AddResidualBlock(
                                new ceres::AutoDiffCostFunction<kinFactor_bet_old_4dof, 4, 4>(old_ctx),
                                NULL,para_kin_local_world_Rt[i][j]
                            );

                            if(j>0){
                                kinFactor_bet_4dof *bet_ctx = new kinFactor_bet_4dof(para_kin_local_world_Rt[i][j], para_kin_local_world_Rt[i][j-1], 1.00, sigma_rt_6dof);

                                problem.AddResidualBlock(
                                    new ceres::AutoDiffCostFunction<kinFactor_bet_4dof, 4, 4,4>(bet_ctx),
                                    NULL,
                                    para_kin_local_world_Rt[i][j], para_kin_local_world_Rt[i][j-1]);
                            }
                            
                        }
                        printf("before %.4lf %.4lf %.4lf %.4lf \n",para_kin_local_world_Rt[i][kin_length/2][0],para_kin_local_world_Rt[i][kin_length/2][1],para_kin_local_world_Rt[i][kin_length/2][2],para_kin_local_world_Rt[i][kin_length/2][3]);
                    }
                }
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    double time = Headers[i];
                    auto iter = kin_2_index.lower_bound(time);
                    if (iter != kin_2_index.end() && abs(time - iter->first) <= 0.02)
                    {
                        int nxt = iter->second;
                        for(int j=1;j<=3;j++){
                            if(AGENT_NUMBER!=j&&kin_can[j][nxt]){
                                kinFactor_connect_4dof_tight *conxt = new kinFactor_connect_4dof_tight(mat_2_world.Ps,
                                mat_2_world.Rs.toRotationMatrix(),
                                kin_mea_ps[j][nxt],kin_mea_qs[j][nxt].toRotationMatrix(),0.04);
                                problem.AddResidualBlock(
                                    new ceres::AutoDiffCostFunction<kinFactor_connect_4dof_tight, 1, 7, 4, 3, 1>(conxt),
                                    NULL,
                                    para_Pose[i],para_kin_local_world_Rt[j][nxt],para_hinge,para_length);
                                resNum += 1;
                            }
                            else if(AGENT_NUMBER==j&&kin_can[j][nxt]&&USE_LOOSE)
                            {
                                kinFactor_connect_4dof_self *conxt = new kinFactor_connect_4dof_self(mat_2_world.Ps,
                                mat_2_world.Rs.toRotationMatrix(),
                                kin_mea_ps[j][nxt],kin_mea_qs[j][nxt].toRotationMatrix(),0.04);
                                problem.AddResidualBlock(
                                    new ceres::AutoDiffCostFunction<kinFactor_connect_4dof_self, 3, 7, 4, 3, 1>(conxt),
                                    NULL,
                                    para_Pose[i],para_kin_local_world_Rt[j][nxt],para_hinge,para_self_len);
                                resNum += 1;

                                //ceres::CostFunction* cost_function = movingError::Create(kin_mea_ps[j][nxt],  0.02);
                                //problem.AddResidualBlock(cost_function, NULL, para_Pose[i]);
                            }
                        }
                        if(kin_can[0][nxt]&&kin_can[x][nxt]&&kin_can[y][nxt]){
                            //printf("%lf ",para_imu_z_val[nxt][0]);
                            //printf("%lf %lf %lf %lf",kin_mea_qs[x][nxt].x(),kin_mea_qs[x][nxt].y(),kin_mea_qs[x][nxt].z(),kin_mea_qs[x][nxt].w());
                            kinFactor_connect_hyp_4dof_tight *conxt2 = new kinFactor_connect_hyp_4dof_tight(
                                mat_2_world.Ps,
                                mat_2_world.Rs.toRotationMatrix(), kin_mea_ps[x][nxt],kin_mea_qs[x][nxt].toRotationMatrix(),
                                kin_mea_ps[y][nxt],kin_mea_qs[y][nxt].toRotationMatrix(), para_imu_z_val[nxt][0], 0.01);
                            problem.AddResidualBlock(
                                    new ceres::AutoDiffCostFunction<kinFactor_connect_hyp_4dof_tight, 1, 7, 4, 4, 3>(conxt2),
                                    NULL,
                                    para_Pose[i], para_kin_local_world_Rt[x][nxt], para_kin_local_world_Rt[y][nxt], para_hinge);
                            resNum+=1;
                        }
                        
                    }//printf("\n");

                    int cnt = pre_integrations[i]->last_t_buf.size();
                    int imu_frame_num=0; // min(cnt, imu_delta_fre);
                    int shift_num;     //= cnt / imu_frame_num;
                    if (cnt == 0 || pre_integrations[i]->sum_dt > 3 || cnt > 300 || IMU_SAEM_FRE == 0)
                    {
                        imu_frame_num = 0;
                        shift_num = 100;
                    }
                    else
                    {
                        imu_frame_num = min(cnt, imu_delta_fre);
                        shift_num = cnt / imu_frame_num;
                    }
                    int tt = 0;
                    for (int j = 1; j < cnt && i >= 1 && imu_frame_num!=0; j += shift_num)
                    {
                        time = pre_integrations[i]->last_t_buf[j];
                        auto iter = kin_2_index.lower_bound(time);
                        if (iter == kin_2_index.end() || abs(time - iter->first) > 0.02)
                            continue;
                        int nxt = iter->second;
                        if(kin_can[0][nxt]&&kin_can[x][nxt]&&kin_can[y][nxt])
                        {
                            Eigen::Vector3d delta_p = pre_integrations[i]->delta_p_buf[j];
                            Eigen::Vector3d delta_v = pre_integrations[i]->delta_v_buf[j];
                            Eigen::Matrix3d delta_q = pre_integrations[i]->delta_q_buf[j].toRotationMatrix();
                            Eigen::Vector3d xworldP, yworldP, eworldP=mat_2_world.Ps;
                            Eigen::Quaterniond xworldR, yworldR, eworldR=mat_2_world.Rs;
                            kinFactor_connect_hyp_4dof_tight_delta *conxt = new kinFactor_connect_hyp_4dof_tight_delta(
                                eworldP, eworldR.toRotationMatrix(),
                                delta_p, delta_q, 
                                kin_mea_ps[x][nxt],kin_mea_qs[x][nxt].toRotationMatrix(),
                                kin_mea_ps[y][nxt],kin_mea_qs[y][nxt].toRotationMatrix(),
                                kin_fre_time[nxt] - Headers[i - 1], para_imu_z_val[nxt][0],0.02
                            );
                            double *residual=new double[2];
                            problem.AddResidualBlock(
                                    new ceres::AutoDiffCostFunction<kinFactor_connect_hyp_4dof_tight_delta, 1, 7, 4, 4, 3>(conxt),
                                    NULL,
                                    para_Pose[i - 1], para_kin_local_world_Rt[x][nxt], para_kin_local_world_Rt[y][nxt],para_hinge);
                                resNum += 1;
                        }
                    }
                }
                

            }
            cout << "kin_length:" << kin_length << "  " << kin_2_index.size() << "  " << resNum << endl;
        }
    }
    //ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("dr.%d--prepare for ceres: %f other size=%d\n", frame_sol_cnt % 2, t_prepare.toc(),other_pose_map[1].size()+other_pose_map[2].size()+other_pose_map[3].size());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    // printf("solver costs: %f \n", t_solver.toc());
    //printf("begin double2vector\n");
    double2vector();
    //printf("end double2vector\n");
    if (frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if (USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if (STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if (imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if (USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
    // printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //
    save_rt();
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];


        if(USE_LONG_WINDOW)
        {
            OdometryVins back(back_P0,Eigen::Quaterniond{back_R0},t_0);
            auto iter = uwb_2_index.lower_bound(t_0);
            if (iter != uwb_2_index.end() && abs(t_0 - iter->first) <= 0.02)
            {
                int nxt = iter->second;
                for(int i=0;i<=3;i++)
                    back.range[i]=uwb_mea[i][nxt];
            }
            else{
                for(int i=0;i<=3;i++)
                    back.range[i]=-1;
            }
            Ps_long.push_back(back);
            if(Ps_long.size()>LONG_WINDOW_SIZE)Ps_long.pop_front();
        }
        

        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if (USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if (USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if (USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();

    
    
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    // printf("predict pts in next frame\n");
    if (frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if (it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            // printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    // printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                    Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                    double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    // return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                     Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                     depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if (STEREO && it_per_frame.is_stereo)
            {

                Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j)
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                         Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                         depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                         Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                         depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
            }
        }
        double ave_err = err / errCnt;
        if (ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);
    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::updateLatestStates()
{
    mPropagate.lock();
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while (!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mPropagate.unlock();
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const map<int, vector<pair<int, Vector4d>>> &image_line, const double header)
{
    // ROS_INFO("new image coming ------------------------------------------");
    // ROS_INFO("Adding feature points %lu %lu", image.size(),image_line.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, image_line, td))
    {
        marginalization_flag = MARGIN_OLD;
        // printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        // printf("non-keyframe\n");
    }
    ROS_INFO("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_INFO("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    // ROS_INFO("Solving %d", frame_count);
    ROS_INFO("number of feature: %d %d", f_manager.getFeatureCount(), f_manager.getLineFeatureCount());
    Headers[frame_count] = header;
    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl
                                                               << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }
    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;
                }
                if (result)
                {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if (STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulateWithLine(frame_count, Ps, Rs, tic, ric);
            if (frame_count == WINDOW_SIZE)
            {
                // f_manager.triangulateWithLine(frame_count, Ps, Rs, tic, ric);
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs);
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                // onlyLineOpt();   // 三角化以后，优化一把
                optimizationwithLine();
                // optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // stereo only initilization
        if (STEREO && !USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if (frame_count == WINDOW_SIZE)
            {
                f_manager.triangulateWithLine(frame_count, Ps, Rs, tic, ric);
                // optimization();
                onlyLineOpt(); // 三角化以后，优化一把
                optimizationwithLine();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        if (frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }
    }
    else
    {
        TicToc t_solve;
        if (!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulateWithLine(frame_count, Ps, Rs, tic, ric);
        onlyLineOpt(); // 三角化以后，优化一把
        optimizationwithLine();
        // optimization();
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        if (!MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }

        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }
}

void Estimator::onlyLineOpt()
{
    ROS_INFO("begin onlyLine opt");
    ROS_INFO("line feature size = %d ", f_manager.getLineFeatureCount());
    // 固定pose， 只优化line的参数，用来调试line的一些参数，看ba优化出来的最好line地图是啥样
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++) // 将窗口内的 p,q 加入优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization); // p,q
        // 固定 pose
        problem.SetParameterBlockConstant(para_Pose[i]);
    }
    for (int i = 0; i < NUM_OF_CAM; i++) // 外参数
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        // 固定 外参数
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    vector2double(); // 将那些保存在 vector向量里的参数 移到 double指针数组里去
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature_line)
    {
        it_per_id.used_num = it_per_id.feature_line_per_frame.size();                                                       // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation)) // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;

        ++feature_index; // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
        /*
        std::cout << para_LineFeature[feature_index][0] <<" "
                << para_LineFeature[feature_index][1] <<" "
                << para_LineFeature[feature_index][2] <<" "
                << para_LineFeature[feature_index][3] <<"\n";
        */
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock(para_LineFeature[feature_index], SIZE_LINE, local_parameterization_line); // p,q

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        for (auto &it_per_frame : it_per_id.feature_line_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                // continue;
            }
            Vector4d obs = it_per_frame.lineobs;                     // 在第j帧图像上的观测
            lineProjectionFactor *f = new lineProjectionFactor(obs); // 特征重投影误差
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[feature_index]);
            f_m_cnt++;
        }
    }
    if (feature_index < 3)
    {
        ROS_INFO("end onlyLine opt because num of feature is small than 3");
        return;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    ROS_INFO("finish  onlyLine opt success");
    // std::cout <<"!!!!!!!!!!!!!onlyLineOpt!!!!!!!!!!!!!\n";
    double2vector();
    // std::cout << summary.FullReport()<<std::endl;

    // f_manager.removeLineOutlier(Rs,Ps,tic,ric);
}
void Estimator::optimizationwithLine()
{

    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if (!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            // ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if (USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if (STEREO && it_per_frame.is_stereo)
            {
                Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    // printf("prepare for ceres: %f \n", t_prepare.toc());

    if (1)
    {
        int line_m_cnt = 0;
        int linefeature_index = -1;
        for (auto &it_per_id : f_manager.feature_line)
        {
            it_per_id.used_num = it_per_id.feature_line_per_frame.size();                                                       // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
            if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation)) // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
                continue;
            ++linefeature_index; // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
            ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
            problem.AddParameterBlock(para_LineFeature[linefeature_index], SIZE_LINE, local_parameterization_line); // p,q
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            for (auto &it_per_frame : it_per_id.feature_line_per_frame)
            {
                imu_j++;
                if (imu_i == imu_j)
                {
                    // continue;
                }
                Eigen::Vector4d obs = it_per_frame.lineobs;              // 在第j帧图像上的观测
                lineProjectionFactor *f = new lineProjectionFactor(obs); // 特征重投影误差
                problem.AddResidualBlock(f, loss_function,
                                         para_Pose[imu_j],
                                         para_Ex_Pose[0],
                                         para_LineFeature[linefeature_index]);
                line_m_cnt++;
            }
        }
        ROS_INFO("lineFactor: %d", line_m_cnt);
    }
    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;
    // if (marginalization_flag == MARGIN_OLD)
    //     options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    // else
    //     options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    // printf("solver costs: %f \n", t_solver.toc());

    double2vector2();
    // printf("frame_count: %d \n", frame_count);

    if (frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();
        int last_mar_cnt = 0;
        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        int imu_mar_cnt = 0;
        if (USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
                imu_mar_cnt++;
            }
        }
        int feature_mar_cnt = 0;
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                        feature_mar_cnt++;
                    }
                    if (STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if (imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                            feature_mar_cnt++;
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                            feature_mar_cnt++;
                        }
                    }
                }
            }
        }
        int line_mar_cnt = 0;
        if (0)
        {
            int linefeature_index = -1;

            for (auto &it_per_id : f_manager.feature_line)
            {
                it_per_id.used_num = it_per_id.feature_line_per_frame.size();                                                       // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
                if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation)) // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
                    continue;
                ++linefeature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0) // 如果这个特征的初始帧 不对应 要marg掉的最老帧0, 那就不用marg这个特征。即marg掉帧的时候，我们marg掉这帧上三角化的那些点
                    continue;
                for (auto &it_per_frame : it_per_id.feature_line_per_frame)
                {
                    imu_j++;
                    std::vector<int> drop_set;
                    if (imu_i == imu_j)
                    { // drop_set = vector<int>{0, 2};   // marg pose and feature,  !!!! do not need marg, just drop they  !!!
                        continue;
                    }
                    else
                    {
                        drop_set = vector<int>{2}; // marg feature
                    }
                    Eigen::Vector4d obs = it_per_frame.lineobs;              // 在第j帧图像上的观测
                    lineProjectionFactor *f = new lineProjectionFactor(obs); // 特征重投影误差

                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_j], para_Ex_Pose[0], para_LineFeature[linefeature_index]},
                                                                                   drop_set); // vector<int>{0, 2} 表示要marg的参数下标，比如这里对应para_Pose[imu_i], para_Feature[feature_index]
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                    line_mar_cnt++;
                }
            }
            ROS_INFO("add marginalization_info lineProjectionFactor");
        }
        TicToc t_pre_margin;
        ROS_INFO("begin marginalization_info preMarginalize   line_marginal_cnt%d %d %d", imu_mar_cnt, feature_mar_cnt, line_mar_cnt);
        marginalization_info->preMarginalize();
        ROS_INFO("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_INFO("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if (USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
}
void Estimator::double2vector2()

{
    // 六自由度优化的时候，整个窗口会在空间中任意优化，这时候我们需要把第一帧在yaw,position上的增量给去掉，因为vins在这几个方向上不可观，他们优化的增量也不可信。
    // 所以这里的操作过程就相当于是 fix 第一帧的 yaw 和 postion, 使得整个轨迹不会在空间中任意飘。
    // 相机姿态需要变化考虑优化以后，把yaw量旋转回去
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]); // 优化之前的0th的姿态
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    // 优化以后的0th的姿态
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5])
                                             .toRotationMatrix());

    // 优化前后，yaw的变化
    double y_diff = origin_R0.x() - origin_R00.x();
    // TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));

    // 由于VI系统的（绝对位置x,y,z,以及yaw）是不可观的。而优化过程中没有固定yaw角，因此yaw会朝着使得误差函数最小的方向优化，但这不一定是正确的。
    // 所以这里把 yaw角的变化量给旋转回去。
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        // Position 也转移到yaw角优化前的 0th坐在的世界坐标下
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) +
                origin_P0;
        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    // 跟yaw没关系，所以不用管优化前后yaw的变化
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5])
                     .toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    // 先把line旋转到相机坐标系下
    Matrix3d Rwow1 = rot_diff;
    Vector3d tw1b(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2]);
    Vector3d twow1 = -Rwow1 * tw1b + origin_P0;

    // std::cout <<"----------\n"<< Rwow1 <<"\n"<<twow1<<std::endl;
    MatrixXd lineorth_vec(f_manager.getLineFeatureCount(), 4);
    ;
    for (int i = 0; i < f_manager.getLineFeatureCount(); ++i)
    {
        Vector4d orth(para_LineFeature[i][0],
                      para_LineFeature[i][1],
                      para_LineFeature[i][2],
                      para_LineFeature[i][3]);

        // 将line_w优化以后的角度变化yaw的变化旋转回去
        Vector6d line_w1 = Utility::orth_to_plk(orth);
        Vector6d line_wo = Utility::plk_to_pose(line_w1, Rwow1, twow1);
        orth = Utility::plk_to_orth(line_wo);

        lineorth_vec.row(i) = orth;
    }
    f_manager.setLineOrth(lineorth_vec, Ps, Rs, tic, ric);
}

void Estimator::inputRange(int id, double t, double dis)
{
    mBuf.lock();
    range_map[id][t] = dis;
    mBuf.unlock();
}
bool Estimator::getRange(int id, double t, double &dis)
{
    // printf("query size--%d",range_map[id].size());
    if (range_map[id].size() == 0)
        return false;
    // printf("query time %lf %lf %lf",range_map[id].begin()->first,t,range_map[id].rbegin()->first);
    auto it = range_map[id].begin();
    it = range_map[id].lower_bound(t);
    if (it == range_map[id].end())
    {
        if (range_map[id].rbegin()->first - t <= 0.025)
        {
            dis = range_map[id].rbegin()->second;
            return true;
        }
        else
        {
            return false;
        }
    }
    else if (it == range_map[id].begin())
    {
        if (t - range_map[id].rbegin()->first <= 0.025)
        {
            dis = range_map[id].rbegin()->second;
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        auto li = std::prev(it);
        dis = li->second + (it->second - li->second) * (t - li->first) / (it->first - li->first);
        // printf("int %lf",dis);
        return true;
    }
}

void Estimator::getPoseAndUWB(int &tot, std::map<double, int> &mp)
{
    //mRT.lock();
    tot = 0;
    for(int i=0;i<=3;i++){
        uwb_mea_sta[i].clear();
    }
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        // printf("%d %d\n",i,pre_integrations[i]->last_t_buf.size());
        int cnt = pre_integrations[i]->last_t_buf.size();
        int imu_frame_num; // min(cnt, imu_delta_fre);
        int shift_num;     //= cnt / imu_frame_num;
        if (cnt == 0 || pre_integrations[i]->sum_dt > 3 || cnt > 300 || IMU_SAEM_FRE == 0)
        {
            imu_frame_num = 1;
            shift_num = 500;
        }
        else
        {
            imu_frame_num = min(cnt, imu_delta_fre) + 1;
            shift_num = cnt / imu_frame_num;
        }
        double time = Headers[i];
        mp[time] = tot;
        uwb_fre_time[tot] = time;
        // eigenTarrarYaw(Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),para_uwb_local_world_Rt[tot]);
        tot++;
        int messageNumber = 1;
        for (int k = 1; k < cnt && i >= 1; k += shift_num)
        {
            messageNumber++;
            time = pre_integrations[i]->last_t_buf[k];
            mp[time] = tot;
            uwb_fre_time[tot] = time;
            // eigenTarrarYaw(Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),para_uwb_local_world_Rt[tot]);
            tot++;
        }
    }
    int range_success_num = 0;
    for (int uwbIdx = lowNum; uwbIdx <= uwbNum; uwbIdx++)
    {
        for (int i = 0; i < tot; i++)
        {
            uwb_can[uwbIdx][i] = getRange(uwbIdx, uwb_fre_time[i], uwb_mea[uwbIdx][i]);
            if(uwb_can[uwbIdx][i]){
                uwb_mea_sta[uwbIdx].update(uwb_mea[uwbIdx][i]);
            }
            range_success_num += uwb_can[uwbIdx][i] == true;
        }
    }
    // ROS_INFO("range_success_num=%d", range_success_num);
    if (DEPEND == 0)
    {
        for (int j = 0; j < tot; j++)
        {
            eigenTarrarYaw(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), para_uwb_local_world_Rt[j]);
        }
        mat_2_world = OdometryVins(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), 1.0);
        to_world_rt_flag = 1;
    }
    else if (DEPEND == 1)
    {
        Eigen::Vector3d imups;
        Eigen::Matrix3d imurs;
        if (to_world_rt_flag == 0)
        {
            for (int i = 1; i <= WINDOW_SIZE; i++)
            {
                Eigen::Vector3d sp = Ps[i];
                Eigen::Matrix3d sr = Rs[i];
                bool flag = getRTformGT(AGENT_NUMBER, Headers[i], imups, imurs, sp, sr);
                //double yaw = Utility::R2ypr(imurs).x();
                // ROS_INFO("imups and yaw %lf %lf %lf %lf",imups.x(),imups.y(),imups.z(),yaw);
                Eigen::Quaterniond q = Eigen::Quaterniond{imurs};
                if (flag)
                {
                    for (int j = 0; j < tot; j++)
                    {

                        other_RT_map[AGENT_NUMBER][uwb_fre_time[j]] = OdometryVins(imups, q, uwb_fre_time[j]);
                    }
                    // for(int j=1;j<8000;j++){
                    //     other_RT_map[AGENT_NUMBER][uwb_fre_time[tot-1]+j*0.02]=OdometryVins(imups,q,uwb_fre_time[tot-1]+j*0.02);
                    // }
                    to_world_rt_flag = 1;
                    mat_2_world = OdometryVins(imups, q, Headers[i]);
                    mat_2_world.getYawAndNorm();
                    break;
                }
            }
            if (to_world_rt_flag == 1)
            {
                for (int uwbIdx = lowNum; uwbIdx <= uwbNum; uwbIdx++)
                {
                    para_UWB_bias[uwbIdx][0]=0;
                    //para_UWB_bias[uwbIdx][0] = (uwb_mea[uwbIdx][0] / 1.8) * 0.1;
                }
            }
        }
        if (to_world_rt_flag == 1)
        {
            for (int i = 0; i < tot; i++)
            {
                bool flag = true;
                imups = mat_2_world.Ps;
                imurs = mat_2_world.Rs;
                // getRTformMap(AGENT_NUMBER,uwb_fre_time[i], imups, imurs);
                if (flag)
                {
                    // ROS_INFO("query calc %lf %lf %lf %lf",uwb_fre_time[i],imups.x(),imups.y(),imups.z());
                    eigenTarrarYaw(imups, imurs, para_uwb_local_world_Rt[i]);
                    // if(i<10)
                    //  ROS_INFO("query para_uwb_local_world RT
                    //  %lf %lf %lf %lf",para_uwb_local_world_Rt[i][0],
                    //  para_uwb_local_world_Rt[i][1],para_uwb_local_world_Rt[i][2],para_uwb_local_world_Rt[i][3]);
                }
                for (int uwbIdx = lowNum; uwbIdx <= uwbNum; uwbIdx++)
                    uwb_can[uwbIdx][i] &= flag;
            }
        }
        else
        {
            memset(uwb_can, 0, sizeof(uwb_can));
        }
    }
    //mRT.unlock();
}

void Estimator::getPoseAndOtherAgent(int &tot, std::map<double, int> &mp)
{
    //mRT.lock();
    if (uwb_length > 0 && uwb_2_index.size() == uwb_length)
    {
        tot = uwb_length;
        mp = uwb_2_index;
        for (int i = 0; i < tot; i++)
        {
            kin_fre_time[i] = uwb_fre_time[i];
        }
    }
    else
    {
        tot = 0;
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            // printf("%d %d\n",i,pre_integrations[i]->last_t_buf.size());
            int cnt = pre_integrations[i]->last_t_buf.size();
            int imu_frame_num; // min(cnt, imu_delta_fre);
            int shift_num;     //= cnt / imu_frame_num;
            if (cnt == 0 || pre_integrations[i]->sum_dt > 3 || cnt > 300 || IMU_SAEM_FRE == 0)
            {
                imu_frame_num = 1;
                shift_num = 500;
            }
            else
            {
                imu_frame_num = min(cnt, imu_delta_fre) + 1;
                shift_num = cnt / imu_frame_num;
            }
            double time = Headers[i];
            mp[time] = tot;
            kin_fre_time[tot] = time;
            tot++;
            int messageNumber = 1;
            for (int k = 1; k < cnt && i >= 1; k += shift_num)
            {
                messageNumber++;
                time = pre_integrations[i]->last_t_buf[k];
                mp[time] = tot;
                kin_fre_time[tot] = time;
                tot++;
            }
        }
    }
    Eigen::Vector3d imups;
    Eigen::Matrix3d imurs;
    for (int i = 0; i < tot; i++)
    {
        OdometryVins query;
        for (int j = 1; j <= 3; j++)
        {
            //cout<<"AGENT : "<<AGENT_NUMBER<<" "<<j<<" "<<other_pose_map[j].size();
            kin_can[j][i] = OdometryVins::queryOdometryMap(other_pose_map[j], kin_fre_time[i], query, 0.025);
            if(kin_can[j][i])
            {
                kin_mea_ps[j][i] = query.Ps;
                //cout<<"AGENT : "<<AGENT_NUMBER<<" "<<j<<" "<<i<<" "<<kin_mea_ps[j][i]<<"  "<<endl;
                kin_mea_qs[j][i] = query.Rs;
                kin_mea_vs[j][i] = query.Vs;
            }
        }
        kin_can[0][i]=OdometryVins::queryOdometryMap(other_pose_map[0], kin_fre_time[i], query, 0.05);
        para_imu_z_val[i][0]=query.Rs.toRotationMatrix()(2,2);
    }
    if (to_world_rt_flag == 1)
    {
        imups = mat_2_world.Ps;
        imurs = mat_2_world.Rs;
        OdometryVins query;
        for (int i = 0; i < tot; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                if (j == AGENT_NUMBER)
                    eigenTarrarYaw(imups, imurs, para_kin_local_world_Rt[AGENT_NUMBER][i]);
                else
                {
                    if(USE_EXR==0||USE_EXR==2)
                        eigenTarrarYaw(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), para_kin_local_world_Rt[j][i]);
                    else
                    {
                        bool flag=OdometryVins::queryOdometryMap(other_RT_map[j], kin_fre_time[i], query, 0.2);
                        query.getYawAndNorm();
                        if(flag==false)
                        eigenTarrarYaw(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), para_kin_local_world_Rt[j][i]);
                        else
                        eigenTarrarYaw(query.Ps, query.Rs, para_kin_local_world_Rt[j][i]);
                    }
                }
            }
        }
    }
    else
    {
        if (DEPEND == 0)
        {
            for (int i = 0; i < tot; i++)
                for (int j = 1; j <= 3; j++)
                    eigenTarrarYaw(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), para_kin_local_world_Rt[j][i]);
            mat_2_world = OdometryVins(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), 1.0);
            to_world_rt_flag = 1;
        }
        else if (DEPEND == 1)
        {
            Eigen::Vector3d imups;
            Eigen::Matrix3d imurs;
            if (to_world_rt_flag == 0)
            {
                for (int i = 1; i <= WINDOW_SIZE; i++)
                {
                    Eigen::Vector3d sp = Ps[i];
                    Eigen::Matrix3d sr = Rs[i];
                    bool flag = getRTformGT(AGENT_NUMBER, Headers[i], imups, imurs, sp, sr);
                    //double yaw = Utility::R2ypr(imurs).x();
                    Eigen::Quaterniond q = Eigen::Quaterniond{imurs};
                    if (flag)
                    {
                        for (int j = 0; j < tot; j++)
                        {

                            other_RT_map[AGENT_NUMBER][kin_fre_time[j]] = OdometryVins(imups, q, kin_fre_time[j]);
                        }
                        to_world_rt_flag = 1;
                        mat_2_world = OdometryVins(imups, q, Headers[i]);
                        mat_2_world.getYawAndNorm();
                        break;
                    }
                }
            }
        }
        if (to_world_rt_flag == 1)
        {
            imups = mat_2_world.Ps;
            imurs = mat_2_world.Rs;
            for (int i = 0; i < tot; i++)
            {
                for (int j = 1; j <= 3; j++)
                    eigenTarrarYaw(imups, imurs, para_kin_local_world_Rt[j][i]);
            }
        }
        else
        {

        }
    }
    //mRT.unlock();
}
void Estimator::eigenTarrarYaw(Eigen::Vector3d x, Eigen::Quaterniond q, double val[])
{
    val[0] = x.x();
    val[1] = x.y();
    val[2] = x.z();
    val[3] = Utility::R2ypr(q.toRotationMatrix()).x();
}
void Estimator::arrayTeigenYaw(double val[], Eigen::Vector3d &x, Eigen::Quaterniond &q)
{
    x = Eigen::Vector3d(val[0], val[1], val[2]);
    q = Eigen::Quaterniond{Utility::ypr2R(Eigen::Vector3d(val[3], 0, 0))};
    q.normalize();
}

void Estimator::eigenTarrarYaw(Eigen::Vector3d x, Eigen::Matrix3d q, double val[])
{
    val[0] = x.x();
    val[1] = x.y();
    val[2] = x.z();
    val[3] = Utility::R2ypr(q).x();
}

void Estimator::arrayTeigenYaw(double val[], Eigen::Vector3d &x, Eigen::Matrix3d &q)
{
    x = Eigen::Vector3d(val[0], val[1], val[2]);
    q = Utility::ypr2R(Eigen::Vector3d(val[3], 0, 0));
    q.normalize();
}

void Estimator::inputGT(int id, OdometryVins tmp)
{
    mBuf.lock();
    gt_map[id][tmp.time] = tmp;
    mBuf.unlock();
}

void Estimator::inputAnchor(int id, OdometryVins tmp)
{
    mBuf.lock();
    for (int i = 0; i <= 2; i++)
    {
        para_UWB_anchor[id][i] = tmp.Ps(i);
    }
    mBuf.unlock();
}

bool Estimator::getRTformGT(int id, double time, Eigen::Vector3d &p, Eigen::Matrix3d &q, Eigen::Vector3d sp, Eigen::Matrix3d sq)
{
    mRT.lock();
    OdometryVins tmp;
    bool flag = OdometryVins::queryOdometryMap(gt_map[id], time, tmp, 0.08);
    if (flag == false)
    {
        mRT.unlock();
        return flag;
    }
    // ROS_INFO("query GT %lf %lf %lf %lf %lf %lf %lf",tmp.Ps.x(),tmp.Ps.y(),tmp.Ps.z(),tmp.Rs.x(),tmp.Rs.y(),tmp.Rs.z(),tmp.Rs.w());

    Eigen::Quaterniond sr{sq};
    // ROS_INFO("query SP %lf %lf %lf %lf %lf %lf %lf",sp.x(),sp.y(),sp.z(),sr.x(),sr.y(),sr.z(),sr.w());
    Eigen::Affine3d affine;
    affine = Eigen::Translation3d(sp) * sr;
    Eigen::Matrix4d matrix = affine.matrix();
    Eigen::Affine3d affine2;
    affine2 = Eigen::Translation3d(tmp.Ps) * tmp.Rs;
    Eigen::Matrix4d matrix2 = affine2.matrix();

    Eigen::Matrix4d matrix3 = matrix2 * matrix.inverse();

    Eigen::Affine3d affine3(matrix3);
    q = (Eigen::Quaterniond(affine3.rotation())).toRotationMatrix();
    p = affine3.translation();
    double yaw = Utility::R2ypr(q).x();
    q = Utility::ypr2R(Eigen::Vector3d(yaw, 0, 0));
    // ROS_INFO("query calc %lf %lf %lf %lf ",p.x(),p.y(),p.z(),yaw);
    mRT.unlock();
    return true;
}
bool Estimator::getRTformMap(int id, double time, Eigen::Vector3d &p, Eigen::Matrix3d &q)
{
    mRT.lock();
    OdometryVins tmp;
    bool flag = OdometryVins::queryOdometryMap(other_RT_map[id], time, tmp, 0.5);
    if (flag)
    {
        p = tmp.Ps;
        q = tmp.Rs.toRotationMatrix();
    }
    mRT.unlock();
    return flag;
}
void Estimator::save_rt()
{
    // for(int i=0;i<uwb_length;i++){
    //     Eigen::Vector3d p;
    //     Eigen::Quaterniond q;
    //     arrayTeigenYaw(para_uwb_local_world_Rt[i],p,q);
    //     double yaw=Utility::R2ypr(q.toRotationMatrix()).x();
    //     q=Eigen::Quaterniond{Utility::ypr2R(Vector3d(yaw,0,0))};
    //     other_RT_map[AGENT_NUMBER][uwb_fre_time[i]]=OdometryVins(p,q,uwb_fre_time[i]);
    // }
    if(USE_EXR==1){
        for(int i=0;i<kin_length;i++){
            for(int j=1;j<=3;j++){
                if(j!=AGENT_NUMBER){
                    Eigen::Vector3d p;
                    Eigen::Quaterniond q;
                    arrayTeigenYaw(para_kin_local_world_Rt[j][i],p,q);
                    other_RT_map[j][kin_fre_time[i]]=OdometryVins(p,q,kin_fre_time[i]);
                }
            }
        }
    }
    
}
void Estimator::clearMap()
{
    mRT.lock();
    for (int i = 1; i <= 3; i++)
    {
        while (other_RT_map[i].size() > 100 && other_RT_map[i].begin()->first - Headers[0] < -0.05)
        {
            other_RT_map[i].erase(other_RT_map[i].begin());
        }
        while (other_pose_map[i].size() > 100 && other_pose_map[i].begin()->first - Headers[0] < -0.05)
        {
            other_pose_map[i].erase(other_pose_map[i].begin());
        }
        while (gt_map[i].size() > 100 && gt_map[i].begin()->first - Headers[0] < -0.05)
        {
            gt_map[i].erase(gt_map[i].begin());
        }
    }
    for (int i = 0; i < 4; i++)
    {

        while (range_map[i].size() > 100 && range_map[i].begin()->first - Headers[0] < -0.05)
        {
            range_map[i].erase(range_map[i].begin());
        }
    }
    while (other_pose_map[0].size() > 100 && other_pose_map[0].begin()->first - Headers[0] < -0.05)
    {
            other_pose_map[0].erase(other_pose_map[0].begin());
    }
    mRT.unlock();
}

void Estimator::inputOtherPose(int id, OdometryVins tmp)
{
    mRT.lock();
    
    //cout<<"Agent_number"<<AGENT_NUMBER<<" "<<id<<other_pose_map[id].size();
    //if(id==2)
    other_pose_map[id][tmp.time] = tmp;
    mRT.unlock();
}