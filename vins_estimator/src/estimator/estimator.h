/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
 
#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/uwb_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/line_parameterization.h"
#include "../factor/line_projection_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"
#include "../featureTracker/feature_tracker_line.h"
#include "../factor/kin_factor.h"
#include "uwb_manager.h"
class Estimator
{
  public:
    Estimator();
    ~Estimator();
    void setParameter();

    // interface
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
    const map<int, vector<pair<int, Vector4d>>> &image_line, const double header);
    void processMeasurements();
    void changeSensorType(int use_imu, int use_stereo);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    void predictPtsInNextFrame();
    void outliersRejection(set<int> &removeIndex);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    void updateLatestStates();
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    bool IMUAvailable(double t);
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);
    void onlyLineOpt();   // 三角化以后，优化一把
    void optimizationwithLine();
    void double2vector2();
    void inputRange(int id,double t,double dis);
    bool getRange(int id,double t,double &dis);
    void getPoseAndUWB(int &tot, std::map<double, int> &mp);
    void getPoseAndOtherAgent(int &tot, std::map<double, int> &mp);
    void eigenTarrarYaw(Eigen::Vector3d x, Eigen::Quaterniond q, double val[]);
    void arrayTeigenYaw(double val[],Eigen::Vector3d &x,Eigen::Quaterniond &q);
    void eigenTarrarYaw(Eigen::Vector3d x, Eigen::Matrix3d q, double val[]);
    void arrayTeigenYaw(double val[],Eigen::Vector3d &x,Eigen::Matrix3d &q);
    void inputGT(int id,OdometryVins tmp);
    bool getRTformGT(int i,double time,Eigen::Vector3d &p,Eigen::Matrix3d &q,Eigen::Vector3d sp,Eigen::Matrix3d sq);
    bool getRTformMap(int i,double time,Eigen::Vector3d &p,Eigen::Matrix3d &q);
    void inputOtherPose(int id,OdometryVins tmp);
    void save_rt();
    void clearMap();
    void inputAnchor(int id,OdometryVins tmp);
    void getIndexByCycle(int &x,int &y){
        if(AGENT_NUMBER==1){
            x=2,y=3;
        }
        else if(AGENT_NUMBER==2){
            x=3,y=1;
        }
        else{
            x=1,y=2;
        }
    }
    //bool queryOdometry(map<double,OdometryVins>mp,double time,OdometryVins &query);
    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    std::mutex mProcess;
    std::mutex mBuf;
    std::mutex mPropagate;
    std::mutex mRT;
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
    queue<pair<double,pair<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> , map<int, vector<pair<int, Vector4d>>> >>> featureWithLineBuf;
    double prevTime, curTime;
    bool openExEstimation;

    std::thread trackThread;
    std::thread processThread;

    FeatureTracker featureTracker;
    FeatureTrackerLine line_feature_tracker;

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;

    Matrix3d ric[2];
    Vector3d tic[2];

    Vector3d        Ps[(WINDOW_SIZE + 1)];
    Vector3d        Vs[(WINDOW_SIZE + 1)];
    Matrix3d        Rs[(WINDOW_SIZE + 1)];
    Vector3d        Bas[(WINDOW_SIZE + 1)];
    Vector3d        Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[2][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];
    double para_LineFeature[NUM_OF_F][SIZE_LINE];
    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    bool initFirstPoseFlag;
    bool initThreadFlag;

    int frame_sol_cnt;
    std::map<double,double>range_map[5];
    std::map<double,OdometryVins>gt_map[5];
    std::map<double,OdometryVins>other_RT_map[5];
    std::map<double,OdometryVins>other_pose_map[5];
    OdometryVins mat_2_world;
    int uwb_length=0;
    int kin_length=0;
    double kin_fre_time[WINDOW_SIZE*15];
    double uwb_fre_time[WINDOW_SIZE*15];
    std::map<double, int> uwb_2_index;
    std::map<double, int> kin_2_index;
    int    uwb_can[5][(WINDOW_SIZE + 1)*15];
    int    kin_can[5][(WINDOW_SIZE + 1)*15];
    double uwb_mea[5][(WINDOW_SIZE + 1)*15];
    double para_imu_z_val[(WINDOW_SIZE+1)*15][1];
    Eigen::Vector3d kin_mea_ps[4][(WINDOW_SIZE + 1)*15];
    Eigen::Vector3d kin_mea_vs[4][(WINDOW_SIZE + 1)*15];
    Eigen::Quaterniond kin_mea_qs[4][(WINDOW_SIZE + 1)*15];
    double para_UWB_anchor[5][3];
    double para_UWB_bias[5][1];
    Eigen::Vector3d UWB_anchor[5];
    double para_uwb_local_world_Rt[(WINDOW_SIZE + 1)*15][SIZE_POSE];
    double para_kin_local_world_Rt[5][(WINDOW_SIZE + 1)*15][SIZE_POSE];
    int to_world_rt_flag;


    double para_hinge[3];
    double para_tag[3];
    double para_length[1];

    OnlineStatistics sta;

    OnlineStatistics uwb_mea_sta[5];



    std::deque<OdometryVins> Ps_long;
    std::deque<Eigen::Vector3d> Ps_long_res;
    double para_Pose_Long[LONG_WINDOW_SIZE][SIZE_POSE];
    double para_self_len[1];
};
