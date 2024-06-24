/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;
int USELINE;
int imu_delta_fre;
int IMU_SAEM_FRE;
int MULAGENT;
int DEPEND;
int USE_UWB;
int AGENT_NUMBER;
int SIM_UE;
int SIM_UWB;
int USE_KIN;
int uwbNum = 0, lowNum = 0;
int FLIGHT_MODE;
int USE_EXR;
int ANCHORNUMBER = 4;
int USE_EST_UWB = 0;
int USE_TRUE_NOISE = 0;
Eigen::Vector3d UWB_TAG;
Eigen::Vector3d HINGE;
double KIN_LENGTH;
Eigen::Matrix<double, 7, 1> sigma_rt_6dof;
Eigen::Matrix<double, 7, 1> sigma_vins_6dof;
int USE_LONG_WINDOW;
double LINK_W, MOVE_W;
int USE_LOOSE;
int USE_GPU;
int USE_GPU_ACC_FLOW;
int UWB_TAG_ID;
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{

    KIN_LENGTH = 0.841;
    sigma_rt_6dof(0) = sigma_rt_6dof(1) = sigma_rt_6dof(2) = 0.01;
    sigma_rt_6dof(3) = sigma_rt_6dof(4) = sigma_rt_6dof(5) = sigma_rt_6dof(6) = 0.04;
    sigma_vins_6dof(0) = sigma_vins_6dof(1) = sigma_vins_6dof(2) = 0.04;
    sigma_vins_6dof(3) = sigma_vins_6dof(4) = sigma_vins_6dof(5) = sigma_vins_6dof(6) = 0.1;
    USE_EXR = 0;
    USE_LONG_WINDOW = 0;
    LINK_W = 0.04;
    MOVE_W = 0.015;
    SIM_UE = 0;
    SIM_UWB = 0;
    MULAGENT = 0;
    DEPEND = 1;
    USE_EST_UWB = 1;

    FILE *fh = fopen(config_file.c_str(), "r");
    if (fh == NULL)
    {
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if (USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else
    {
        if (ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if (NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);

    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if (fsSettings["use_gpu"].type() != cv::FileNode::NONE)
        USE_GPU = fsSettings["use_gpu"];
    if (fsSettings["use_gpu_acc_flow"].type() != cv::FileNode::NONE)
        USE_GPU_ACC_FLOW = fsSettings["use_gpu_acc_flow"];
    if (NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib;
        // printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);

        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if (!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    USE_LOOSE = 0;
    USE_KIN = 0;
    if (fsSettings["use_kin"].type() != cv::FileNode::NONE)
        USE_KIN = fsSettings["use_kin"];
    USELINE = 0;
    USE_UWB = 0;
    if (fsSettings["use_uwb"].type() != cv::FileNode::NONE)
        USE_UWB = fsSettings["use_uwb"];

    IMU_SAEM_FRE = 1;
    imu_delta_fre = 3;
    if (fsSettings["imu_fre"].type() != cv::FileNode::NONE)
        IMU_SAEM_FRE = fsSettings["imu_fre"];
    imu_delta_fre = 3;
    if (fsSettings["imu_delta"].type() != cv::FileNode::NONE)
        imu_delta_fre = fsSettings["imu_delta"];

    AGENT_NUMBER = fsSettings["agent_number"];

    if (AGENT_NUMBER == 1)
    {
        uwbNum = 3, lowNum = 0;
    }
    else if (AGENT_NUMBER == 2)
        uwbNum = 3, lowNum = 0;
    else
        uwbNum = 3, lowNum = 0;

    if (USE_UWB && SIM_UWB == 0 && SIM_UE == 0)
    {
        if (fsSettings["uwb_tag_id"].type() != cv::FileNode::NONE)
            UWB_TAG_ID = fsSettings["uwb_tag_id"];
        else
        {
            ROS_INFO("please input uwb tag id in config yaml");
            assert(0);
        }
        UWB_TAG << -0.1, 0.00, 0.03;
        if (fsSettings["body_T_tag"].type() != cv::FileNode::NONE)
        {
            cv::Mat cv_T;
            fsSettings["body_T_tag"] >> cv_T;
            Eigen::Matrix4d T;
            cv::cv2eigen(cv_T, T);
            UWB_TAG(0) = T(3, 0);
            UWB_TAG(1) = T(3, 1);
            UWB_TAG(2) = T(3, 2);
        }
    }
    HINGE << -0.1, 0.00, -0.03;
    if (fsSettings["body_T_hinge"].type() != cv::FileNode::NONE)
    {
        cv::Mat cv_T;
        fsSettings["body_T_hinge"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        HINGE(0) = T(3, 0);
        HINGE(1) = T(3, 1);
        HINGE(2) = T(3, 2);
    }
    
    fsSettings.release();
}
