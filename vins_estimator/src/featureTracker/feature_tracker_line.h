/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include <opencv2/features2d.hpp>
#include "line_descriptor_custom.hpp"
using namespace std;
using namespace camodocal;
using namespace Eigen;
using namespace cv::line_descriptor;
using namespace cv;


struct Line
{
	Point2f StartPt;
	Point2f EndPt;
	float lineWidth;
	Point2f Vp;

	Point2f Center;
	Point2f unitDir; // [cos(theta), sin(theta)]
	float length;
	float theta;

	// para_a * x + para_b * y + c = 0
	float para_a;
	float para_b;
	float para_c;

	float image_dx;
	float image_dy;
    float line_grad_avg;

	float xMin;
	float xMax;
	float yMin;
	float yMax;
	unsigned short id;
	int colorIdx;
};

class FrameLines
{
public:
    int frame_id;
    Mat img;
    
    vector<Line> vecLine;

    vector< int > lineID;

    // opencv3 lsd+lbd
    std::vector<KeyLine> keylsd;
    
    Mat lbd_descr;
};
typedef shared_ptr< FrameLines > FrameLinesPtr;


class FeatureTrackerLine
{
    public:
    FeatureTrackerLine();
    map<int, vector<pair<int, Eigen::Vector4d>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void readIntrinsicParameter(const vector<string> &calib_file);
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight);
    cv::Mat getTrackImage();
    vector<camodocal::CameraPtr> m_camera;
    cv::Mat undist_map1_, undist_map2_ , K_;
    int frame_cnt;
    FrameLinesPtr curframe_, forwframe_;
    vector<int> ids;                     // 每个特征点的id
    vector<int> linetrack_cnt;           // 记录某个特征已经跟踪多少帧了，即被多少帧看到了
    int allfeature_cnt;                  // 用来统计整个地图中有了多少条线，它将用来赋值
    int n_id;
    bool stereo_cam;
    bool hasPrediction;
    double sum_time;
    double mean_time;
    cv::Mat imTrack;
    map<int,int>line_feature_id_use;
};