/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}
int FeatureLinePerId::endFrame()
{
    return start_frame + feature_line_per_frame.size() - 1;
}
FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}


bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    new_feature_num = 0;
    long_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }

    //if (frame_count < 2 || last_track_num < 20)
    //if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
    const map<int, vector<pair<int,Eigen::Vector4d> > > &image_line ,double td)
{
    bool addFeatureCheckParallaxOnlyPoint=addFeatureCheckParallax(frame_count,image,td);

    double parallax_line_sum = 0;
    int parallax_line_num = 0;
    last_track_line_num = 0;
    last_average_line_parallax = 0;
    new_feature_line_num = 0;
    long_track_line_num = 0;
    //ROS_INFO("input feature: %d", (int)image_line.size());
    //ROS_DEBUG("num of feature: %d", getFeatureCount());
    int f1=0,f2=0;
    int num=0;
    //for(auto &id : feature_line)num+=id.feature_line_per_frame.size();
    //printf("load map ");
    for(auto &id_lts:image_line){
        FeatureLinePerFrame f_per_fra(id_lts.second[0].second,td);
        //assert(id_lts.second[0].first == 0);
        //printf("%d ",image_line.second.size());
        if(id_lts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_lts.second[1].second);
            // if(id_lts.second[1].first!=1){
            //     printf("frame_cnt= %d feature_id = %d  vector_size=%d camera_id = %d %d\n",frame_count,id_lts.first,id_lts.second.size(),
            //     id_lts.second[0].first,id_lts.second[1].first);
            // }
            assert(id_lts.second[1].first == 1);
        }
        //printf("%d ",id_lts.first);
        int feature_id = id_lts.first;
        auto it = find_if(feature_line.begin(), feature_line.end(), [feature_id](const FeatureLinePerId &it)
        {
            return it.feature_id == feature_id;
        });
        if (it == feature_line.end())
        {
            feature_line.push_back(FeatureLinePerId(feature_id, frame_count));
            feature_line.back().feature_line_per_frame.push_back(f_per_fra);
            new_feature_line_num++;
            f1++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_line_per_frame.push_back(f_per_fra);
            last_track_line_num++;
            it->all_obs_cnt++;
            if( it-> feature_line_per_frame.size() >= 4)
                long_track_line_num++;
            f2++;
        }
    }
    //printf("\n");
    //for(auto &id : feature_line)num+=id.feature_line_per_frame.size();
    //cout<<endl<<"f1: "<<f1<<"  f2:"<<f2<<" diff="<<num<<endl;
    bool addFeatureCheckParallaxOnlyLine=false;
    if (frame_count < 2 || last_track_line_num < 20 || long_track_line_num < 40 || new_feature_line_num > 0.5 * last_track_line_num)
        addFeatureCheckParallaxOnlyLine=true;
    for (auto &it_line_per_id : feature_line)
    {
        if (it_line_per_id.start_frame <= frame_count - 2 &&
            it_line_per_id.start_frame + int(it_line_per_id.feature_line_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_line_sum += compensatedParallaxLine(it_line_per_id, frame_count);
            parallax_line_num++;
        }
    }
    return addFeatureCheckParallaxOnlyPoint;
}
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}


void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{

    if(frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        for (auto &it_per_id : feature)
        {
            if (it_per_id.estimated_depth > 0)
            {
                int index = frameCnt - it_per_id.start_frame;
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); 
                }
            }
        }
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
        RCam = Rs[frameCnt - 1] * ric[0];
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

        if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
        {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[0].transpose(); 
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;

            Eigen::Quaterniond Q(Rs[frameCnt]);
            //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
    }
}

void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;

        if(STEREO && it_per_id.feature_per_frame[0].is_stereo)
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;
            //cout << "left pose " << leftPose << endl;

            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            //cout << "right pose " << rightPose << endl;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
            //cout << "point0 " << point0.transpose() << endl;
            //cout << "point1 " << point1.transpose() << endl;

            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        else if(it_per_id.feature_per_frame.size() > 1)
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            imu_i++;
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[1].point.head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}

void FeatureManager::triangulateWithLine(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    triangulate(frameCnt,Ps,Rs,tic,ric);
    int c[5]={0,0,0,0,0};
    int num=0;
    for(auto &id : feature_line)num+=id.feature_line_per_frame.size();
    //cout<<"traingulateWithLine :"<<num<<" min idx"<<feature_line.begin().feature_id<<" "<<feature_line.rbegin().feature_id<<endl;
    for (auto &it_per_id : feature_line)        // 遍历每个特征，对新特征进行三角化
    {
        if (it_per_id.is_triangulation)       // 如果已经三角化了
        {
            c[0]++;
            continue;
        }
        it_per_id.used_num = it_per_id.feature_line_per_frame.size();    // 已经有多少帧看到了这个特征

        if(STEREO && it_per_id.used_num>=2&&it_per_id.feature_line_per_frame[0].is_stereo){

            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;
            //cout << "left pose " << leftPose << endl;
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            Eigen::Vector3d baseline=t1-t0;
            Eigen::Vector4d lineobs_l,lineobs_r;
            FeatureLinePerFrame it_per_frame = it_per_id.feature_line_per_frame.front();
            lineobs_l = it_per_frame.lineobs;
            lineobs_r = it_per_frame.lineobs_R;
            // plane pi from ith left obs in ith left camera frame
            Eigen::Vector3d p1( lineobs_l(0), lineobs_l(1), 1 );
            Eigen::Vector3d p2( lineobs_l(2), lineobs_l(3), 1 );
            Eigen::Vector4d pii = Utility::pi_from_ppp(p1, p2,Eigen::Vector3d( 0, 0, 0 ));

            // plane pi from ith right obs in ith left camera frame
            Eigen::Vector3d p3( lineobs_r(0) + baseline(0), lineobs_r(1)+baseline(1), 1 );
            Eigen::Vector3d p4( lineobs_r(2) + baseline(0), lineobs_r(3)+baseline(1), 1 );
            Eigen::Vector4d pij = Utility::pi_from_ppp(p3, p4,Eigen::Vector3d(baseline(0), baseline(1), 0));

            Vector6d plk = Utility::pipi_plk( pii, pij );
            Eigen::Vector3d n = plk.head(3);
            Eigen::Vector3d v = plk.tail(3);
            it_per_id.line_plucker = plk;  // plk in camera frame
            c[1]++;
            it_per_id.is_triangulation = true;
        }
        else{
            if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2))   // 看到的帧数少于2， 或者 这个特征最近倒数第二帧才看到， 那都不三角化
            {
                //printf("used_num %d start_frame %d , ",it_per_id.used_num,it_per_id.start_frame);
                if(it_per_id.used_num<LINE_MIN_OBS)
                c[2]++;
                else
                c[3]++;
                continue;
            }
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

            double d = 0, min_cos_theta = 1.0;
            Eigen::Vector3d tij;
            Eigen::Matrix3d Rij;
            Eigen::Vector4d obsi,obsj;  // obs from two frame are used to do triangulation

            // plane pi from ith obs in ith camera frame
            Eigen::Vector4d pii;
            Eigen::Vector3d ni;      // normal vector of plane    
            for (auto &it_per_frame : it_per_id.feature_line_per_frame)   // 遍历所有的观测， 注意 start_frame 也会被遍历
            {
                imu_j++;
                if(imu_j == imu_i)   // 第一个观测是start frame 上
                {
                    obsi = it_per_frame.lineobs;
                    Eigen::Vector3d p1( obsi(0), obsi(1), 1 );
                    Eigen::Vector3d p2( obsi(2), obsi(3), 1 );
                    pii = Utility::pi_from_ppp(p1, p2,Eigen::Vector3d( 0, 0, 0 ));
                    ni = pii.head(3); ni.normalize();
                    continue;
                }
                // 非start frame(其他帧)上的观测
                Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
                Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

                Eigen::Vector3d t = R0.transpose() * (t1 - t0);   // tij
                Eigen::Matrix3d R = R0.transpose() * R1;          // Rij
                
                Eigen::Vector4d obsj_tmp = it_per_frame.lineobs;
                // plane pi from jth obs in ith camera frame
                Eigen::Vector3d p3( obsj_tmp(0), obsj_tmp(1), 1 );
                Eigen::Vector3d p4( obsj_tmp(2), obsj_tmp(3), 1 );
                p3 = R * p3 + t;
                p4 = R * p4 + t;
                Eigen::Vector4d pij = Utility::pi_from_ppp(p3, p4,t);
                Eigen::Vector3d nj = pij.head(3); nj.normalize(); 
                double cos_theta = ni.dot(nj);
                if(cos_theta < min_cos_theta)
                {
                    min_cos_theta = cos_theta;
                    tij = t;
                    Rij = R;
                    obsj = obsj_tmp;
                    d = t.norm();
                }
            }
            if(min_cos_theta > 0.998) 
            {
                c[3]++;
                continue;
            }
            // plane pi from jth obs in ith camera frame
            Vector3d p3( obsj(0), obsj(1), 1 );
            Vector3d p4( obsj(2), obsj(3), 1 );
            p3 = Rij * p3 + tij;
            p4 = Rij * p4 + tij;
            Vector4d pij = Utility::pi_from_ppp(p3, p4,tij);

            Vector6d plk = Utility::pipi_plk( pii, pij );
            Vector3d n = plk.head(3);
            Vector3d v = plk.tail(3);
            it_per_id.line_plucker = plk;  // plk in camera frame
            it_per_id.is_triangulation = true;
            c[4]++;
        }
        
    }
    //printf("\n");
    cout<<c[0]<<" "<<c[1]<<" "<<c[2]<<" "<<c[3]<<" "<<c[4]<<"  "<<feature_line.size()<<endl;
    //removeLineOutlier();
}
void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            feature.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
    if(USELINE){
        for (auto it = feature_line.begin(), it_next = feature_line.begin();
         it != feature_line.end(); it = it_next)
        {
            it_next++;

            if (it->start_frame != 0)    // 如果特征不是在这帧上初始化的，那就不用管，只要管id--
            {
                it->start_frame--;
            }
            else{
                it->feature_line_per_frame.erase(it->feature_line_per_frame.begin());  // 移除观测
                if (it->feature_line_per_frame.size() <= 0)                     // 如果观测到这个帧的图像少于两帧，那这个特征不要了
                {
                    feature_line.erase(it);
                    continue;
                }
                else  // 如果还有很多帧看到它，而我们又把这个特征的初始化帧给marg掉了，那就得把这个特征转挂到下一帧上去, 这里 marg_R, new_R 都是相应时刻的相机坐标系到世界坐标系的变换
                {
                    it->removed_cnt++;
                    // transpose this line to the new pose
                    Matrix3d Rji = new_R.transpose() * marg_R;     // Rcjw * Rwci
                    Vector3d tji = new_R.transpose() * (marg_P - new_P);
                    Vector6d plk_j = Utility::plk_to_pose(it->line_plucker, Rji, tji);
                    it->line_plucker = plk_j;
                }
            }
        }
    }
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }

    if(USELINE){
        for (auto it = feature_line.begin(), it_next = feature_line.begin();
         it != feature_line.end(); it = it_next)
        {
            it_next++;

            // 如果这个特征不是在窗口里最老关键帧上观测到的，由于窗口里移除掉了一个帧，所有其他特征对应的初始化帧id都要减1左移
            // 例如： 窗口里有 0,1,2,3,4 一共5个关键帧，特征f2在第2帧上三角化的， 移除掉第0帧以后， 第2帧在窗口里的id就左移变成了第1帧，这是很f2的start_frame对应减1
            if (it->start_frame != 0)
                it->start_frame--;
            else
            {
                it->feature_line_per_frame.erase(it->feature_line_per_frame.begin());  // 删掉特征ft在这个图像帧上的观测量
                if (it->feature_line_per_frame.size() == 0)                       // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                    feature_line.erase(it);
            }
        }
    }
    
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
    if(USELINE){
        for (auto it = feature_line.begin(), it_next = feature_line.begin(); it != feature_line.end(); it = it_next)
        {
            it_next++;

            if (it->start_frame == frame_count)  // 由于要删去的是第frame_count-1帧，最新这一帧frame_count的id就变成了i-1
            {
                it->start_frame--;
            }
            else
            {
                int j = WINDOW_SIZE - 1 - it->start_frame;    // j指向第i-1帧
                if (it->endFrame() < frame_count - 1)
                    continue;
                it->feature_line_per_frame.erase(it->feature_line_per_frame.begin() + j);   // 删掉特征ft在这个图像帧上的观测量
                if (it->feature_line_per_frame.size() == 0)                            // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                    feature_line.erase(it);
            }
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

double FeatureManager::compensatedParallaxLine(const FeatureLinePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeatureLinePerFrame &frame_i = it_per_id.feature_line_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeatureLinePerFrame &frame_j = it_per_id.feature_line_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    // Vector3d p_j(frame_j.line.StartPt.x,frame_j.line.StartPt.y,0);

    // double u_j = p_j(0);
    // double v_j = p_j(1);

    // Vector3d p_i(frame_i.line.StartPt.x,frame_i.line.StartPt.y,0);
    // Vector3d p_i_comp;

    // //int r_i = frame_count - 2;
    // //int r_j = frame_count - 1;
    // //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    // p_i_comp = p_i;
    // double dep_i = p_i(2);
    // double u_i = p_i(0) / dep_i;
    // double v_i = p_i(1) / dep_i;
    // double du = u_i - u_j, dv = v_i - v_j;

    // double dep_i_comp = p_i_comp(2);
    // double u_i_comp = p_i_comp(0) / dep_i_comp;
    // double v_i_comp = p_i_comp(1) / dep_i_comp;
    // double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    // ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));
    return ans;
}

void FeatureManager::removeLineOutlier()
{
    for (auto it_per_id = feature_line.begin(), it_next = feature_line.begin();
         it_per_id != feature_line.end(); it_per_id = it_next)
    {
        it_next++;
        it_per_id->used_num = it_per_id->feature_line_per_frame.size();
        // TODO: 右目没看到
        if (it_per_id->is_triangulation || it_per_id->used_num < 2)  // 已经三角化了 或者 少于两帧看到 或者 右目没有看到
            continue;
        int imu_i = it_per_id->start_frame, imu_j = imu_i -1;
        Eigen::Vector3d pc, nc, vc;
        nc = it_per_id->line_plucker.head(3);
        vc = it_per_id->line_plucker.tail(3);
        Eigen::Matrix4d Lc;
        Lc << Utility::skewSymmetric(nc), vc, -vc.transpose(), 0;

        Eigen::Vector4d obs_startframe = it_per_id->feature_line_per_frame[0].lineobs;   // 第一次观测到这帧
        Eigen::Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Eigen::Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Eigen::Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Eigen::Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Eigen::Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Eigen::Vector3d cam = Vector3d( 0, 0, 0 );

        Eigen::Vector4d pi1 = Utility::pi_from_ppp(cam, p11, p12);
        Eigen::Vector4d pi2 = Utility::pi_from_ppp(cam, p21, p22);

        Eigen::Vector4d e1 = Lc * pi1;
        Eigen::Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

        if(e1(2) < 0 || e2(2) < 0)
        {
            feature_line.erase(it_per_id);
            continue;
        }

        if((e1-e2).norm() > 10)
        {
            feature_line.erase(it_per_id);
            continue;
        }
    }
}
void FeatureManager::removeLineOutlier(Matrix3d Rs2[],Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    ROS_INFO("begin removeLineOutlier with Rs2 Ps tic ric");
    for (auto it_per_id = feature_line.begin(), it_next = feature_line.begin();
         it_per_id != feature_line.end(); it_per_id = it_next)
    {
        it_next++;
        it_per_id->used_num = it_per_id->feature_line_per_frame.size();
        if (!(it_per_id->used_num >= LINE_MIN_OBS && it_per_id->start_frame < WINDOW_SIZE - 2 && it_per_id->is_triangulation))
            continue;

        int imu_i = it_per_id->start_frame, imu_j = imu_i -1;
        Eigen::Vector3d twc = Ps[imu_i] + Rs2[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs2[imu_i] * ric[0];               // Rwc = Rwi * Ric
        // 计算初始帧上线段对应的3d端点
        Eigen::Vector3d pc, nc, vc;
        nc = it_per_id->line_plucker.head(3);
        vc = it_per_id->line_plucker.tail(3);

        Eigen::Matrix4d Lc;
        Lc << Utility::skewSymmetric(nc), vc, -vc.transpose(), 0;

        Eigen::Vector4d obs_startframe = it_per_id->feature_line_per_frame[0].lineobs;   // 第一次观测到这帧
        Eigen::Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Eigen::Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Eigen::Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Eigen::Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Eigen::Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Eigen::Vector3d cam = Vector3d( 0, 0, 0 );

        Eigen::Vector4d pi1 = Utility::pi_from_ppp(cam, p11, p12);
        Eigen::Vector4d pi2 = Utility::pi_from_ppp(cam, p21, p22);

        Eigen::Vector4d e1 = Lc * pi1;
        Eigen::Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

        //std::cout << "line endpoint: "<<e1 << "\n "<< e2<<"\n";
        if(e1(2) < 0 || e2(2) < 0)
        {
            feature_line.erase(it_per_id);
            continue;
        }
        if((e1-e2).norm() > 10)
        {
            feature_line.erase(it_per_id);
            continue;
        }
    }
    ROS_INFO("end removeLineOutlier with Rs2 Ps tic ric");
}
int FeatureManager::getLineFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature_line)
    {

        it.used_num = it.feature_line_per_frame.size();

        if (it.used_num >= LINE_MIN_OBS && it.start_frame < WINDOW_SIZE - 2 && it.is_triangulation)
        {
            cnt++;
        }
    }
    return cnt;
}
MatrixXd FeatureManager::getLineOrthVectorInCamera()
{
    Eigen::MatrixXd lineorth_vec(getLineFeatureCount(),4);
    int feature_index = -1;
    for (auto &it_per_id : feature_line)
    {
        it_per_id.used_num = it_per_id.feature_line_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        lineorth_vec.row(++feature_index) = Utility::plk_to_orth(it_per_id.line_plucker);

    }
    return lineorth_vec;
}
Eigen::MatrixXd FeatureManager::getLineOrthVector(Eigen::Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    Eigen::MatrixXd lineorth_vec(getLineFeatureCount(),4);
    int feature_index = -1;
    for (auto &it_per_id : feature_line)
    {
        it_per_id.used_num = it_per_id.feature_line_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        int imu_i = it_per_id.start_frame;

        //ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

        Vector6d line_w = Utility::plk_to_pose(it_per_id.line_plucker, Rwc, twc);  // transfrom to world frame
        // line_w.normalize();
        lineorth_vec.row(++feature_index) = Utility::plk_to_orth(line_w);
        //lineorth_vec.row(++feature_index) = plk_to_orth(it_per_id.line_plucker);

    }
    return lineorth_vec;
}


void FeatureManager::setLineOrthInCamera(MatrixXd x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature_line)
    {
        it_per_id.used_num = it_per_id.feature_line_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;
        Eigen::Vector4d line_orth = x.row(++feature_index);
        it_per_id.line_plucker = Utility::orth_to_plk(line_orth);// transfrom to camera frame

    }
}
void FeatureManager::setLineOrth(MatrixXd x,Vector3d P[], Matrix3d R[], Vector3d tic[], Matrix3d ric[])
{
    int feature_index = -1;
    for (auto &it_per_id : feature_line)
    {
        it_per_id.used_num = it_per_id.feature_line_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        Eigen::Vector4d line_orth_w = x.row(++feature_index);
        Vector6d line_w = Utility::orth_to_plk(line_orth_w);

        int imu_i = it_per_id.start_frame;
        //ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = P[imu_i] + R[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = R[imu_i] * ric[0];               // Rwc = Rwi * Ric

        it_per_id.line_plucker = Utility::plk_from_pose(line_w, Rwc, twc); // transfrom to camera frame
    }
}