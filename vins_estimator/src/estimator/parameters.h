/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <random>
#include <iomanip>
#include <geometry_msgs/PoseArray.h>
#include <boost/bind.hpp>
using namespace std;

const double FOCAL_LENGTH = 460.0;
const int LINE_MIN_OBS = 4;
const int WINDOW_SIZE = 10;
const int LONG_WINDOW_SIZE=60;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string OUTPUT_FOLDER;
extern std::string IMU_TOPIC;
extern double TD;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern int ROW, COL;
extern int NUM_OF_CAM;
extern int STEREO;
extern int USE_IMU;
extern int MULTIPLE_THREAD;
// pts_gt for debug purpose;
extern map<int, Eigen::Vector3d> pts_gt;

extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int FLOW_BACK;
extern int USELINE;
extern int USE_UWB;
extern int AGENT_NUMBER;
extern int imu_delta_fre;
extern int IMU_SAEM_FRE;
extern int MULAGENT;
extern int DEPEND;
extern int SIM_UE;
extern int USE_KIN;
extern int uwbNum,lowNum;
extern int FLIGHT_MODE;
extern Eigen::Vector3d HINGE;
extern double KIN_LENGTH;
extern int SIM_UWB;
extern Eigen::Matrix<double,7,1>sigma_rt_6dof;
extern Eigen::Matrix<double,7,1>sigma_vins_6dof;
extern int USE_EXR;
extern int USE_LONG_WINDOW;
extern double LINK_W,MOVE_W;
extern int USE_LOOSE;
extern int ANCHORNUMBER;
extern int USE_EST_UWB;
extern int USE_TRUE_NOISE;
extern int USE_GPU;
extern int USE_GPU_ACC_FLOW;
extern int UWB_TAG_ID;
void readParameters(std::string config_file);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1,
    SIZE_LINE = 4
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
