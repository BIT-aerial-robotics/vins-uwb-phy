#ifndef UWB_MANAGER_H
#define UWB_MANAGER_H
#pragma once
#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
#include<deque>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <ros/console.h>
#include <ros/assert.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"
using namespace std;
using namespace Eigen;
class OnlineStatistics {
public:
    OnlineStatistics() : n_(0), mean_(0.0), m2_(0.0) {}

    void update(double value) {
        n_++;
        double delta = value - mean_;
        mean_ += delta / n_;
        double delta2 = value - mean_;
        m2_=((n_ - 1)*1.00 / (n_*n_)*1.00 )* (delta*delta) + ((n_ - 1)*1.00/(n_)*1.00)*m2_;
    }

    double mean() const {
        return mean_;
    }

    double variance() const {
        if (n_ < 2) {
            // 方差至少需要两个样本
            return std::numeric_limits<double>::quiet_NaN();
        }
        return m2_;
    }
    void clear()
    {
        n_=0;
        mean_=0;
        m2_=0;
    }

private:
    int n_;          // 样本数量
    double mean_;    // 均值
    double m2_;      // 用于计算方差的中间变量
};
class ButterworthLowPassFilter {
public:
    ButterworthLowPassFilter(double cutoffFrequency, double sampleRate, int order) 
        : cutoffFrequency_(cutoffFrequency), sampleRate_(sampleRate), order_(order) {
        calculateCoefficients();
        reset();
    }

    double filter(double input) {
        // 巴特沃斯低通滤波差分方程
        double output = b_[0] * input + b_[1] * x_[0] + b_[2] * x_[1]
                      - a_[1] * y_[0] - a_[2] * y_[1];

        // 更新输入和输出缓冲区
        x_[1] = x_[0];
        x_[0] = input;
        y_[1] = y_[0];
        y_[0] = output;

        return output;
    }

    void reset() {
        // 重置输入和输出缓冲区
        x_[0] = x_[1] = 0.0;
        y_[0] = y_[1] = 0.0;
    }

private:
    double cutoffFrequency_;  // 截止频率
    double sampleRate_;      // 采样率
    int order_;              // 滤波器阶数

    // 巴特沃斯低通滤波器系数
    double b_[3];
    double a_[3];

    // 输入和输出缓冲区
    double x_[2];
    double y_[2];

    // 计算巴特沃斯低通滤波器系数
    void calculateCoefficients() {
        double omegaC = 2.0 * M_PI * cutoffFrequency_ / sampleRate_;
        double alpha = std::sin(omegaC) / (2.0 * std::sin(omegaC / (2.0)));

        double cosOmegaC = std::cos(omegaC);

        b_[0] = (1.0 - cosOmegaC) / 2.0;
        b_[1] = 1.0 - cosOmegaC;
        b_[2] = (1.0 - cosOmegaC) / 2.0;

        a_[0] = 1.0 + alpha;
        a_[1] = -2.0 * cosOmegaC;
        a_[2] = 1.0 - alpha;
    }
};
class OdometryVins
{
public:
    OdometryVins(){
        Ps=Eigen::Vector3d(0,0,0);
        Vs=Eigen::Vector3d(0,0,0);
        Ws=Eigen::Vector3d(0,0,0);
        Rs=Eigen::Quaterniond(1,0,0,0);
    }
    OdometryVins(Eigen::Vector3d ps,Eigen::Vector3d vs,Eigen::Vector3d _Ws,Eigen::Quaterniond _Rs,double _time):Ps(ps),Vs(vs),Ws(_Ws),Rs(_Rs),time(_time){}
    OdometryVins(Eigen::Vector3d ps,Eigen::Quaterniond _Rs,double _time):Ps(ps),Rs(_Rs),time(_time){
        Vs=Eigen::Vector3d(0,0,0);
        Ws=Eigen::Vector3d(0,0,0);
    }
    void updateRange(double _range[]);
    OdometryVins interpolation(OdometryVins nx,double t);
    OdometryVins predict(double t);
    double time;
    Eigen::Vector3d Ps,Vs,Ws;
    Eigen::Quaterniond Rs;
    double range[10];
    double getYawAndNorm()
    {
        Eigen::Matrix3d rs=Rs.toRotationMatrix();
        double yaw=Utility::R2ypr(rs).x();
        rs=Utility::ypr2R(Eigen::Vector3d(yaw,0,0));
        Rs=Eigen::Quaterniond{rs};
        return yaw;
    }
    static bool queryOdometryMap(map<double,OdometryVins>&mp,double time,OdometryVins &query,double the)
    {
        if(mp.size()==0)return false;
        auto iter=mp.lower_bound(time);
        if(iter==mp.end()){
            if(abs(mp.rbegin()->first-time)>the)return false;
            else{
                //printf("end()");
                query=mp.rbegin()->second.predict(time);
            }
        }
        else if(iter==mp.begin())
        {
            //printf("begin()");
            if(abs(iter->first-time)>the)return false;
            else{
                query=iter->second.predict(time);
            }
        }
        else{
            //printf("mid() %lf ",time);
            if(abs(iter->first-time)>the)return false;
            else{
                auto last=std::prev(iter);
                query=last->second.interpolation(iter->second,time);
            }
        }return true;
    }
};
class comp {
public:
    bool operator()(const OdometryVins& i, const double& j) {
        return i.time > j;
    }
};


class UWBMeasurement{
    public:
    UWBMeasurement():idx(0),range(0),time(0){}
    UWBMeasurement(int _idx,double _range,double _time):idx(_idx),range(_range),time(_time){}
    int idx;
    double range;
    double time;

};
class UWBManager
{
    public:

    UWBManager();
    UWBManager(int _uwb_num,vector<Eigen::Vector3d> _uwb_loc);
    void clearState();
    bool addUWBMeasurements(int uwb_idx,double time,double distance);
    void addOdometryMeasurements(Eigen::Vector3d Ps,Eigen::Vector3d Vs,Eigen::Vector3d Ws,Eigen::Quaterniond Rs,double time);
    bool query(OdometryVins &x,double time);
    void gaussSmoothen(vector<UWBMeasurement>&values,vector<double>&out);
    void gaussKernel(vector<double>&kernal);
    void smoothRange(int idx);
    double accumulate(int idx);
    double pre_sum[10];
    double OFFSET_THRESH;
    double now_offset;
    double offset_time;
    double sigma;
    int sample;
    int range_len[10];
    int uwb_num;
    int last_smooth_idx[10];
    int RANGE_SIZE;
    bool range_updated;
    bool publish_smooth_range;
    deque<OdometryVins> odometry;
    vector<Eigen::Vector3d> uwb_loc;
    vector<deque<UWBMeasurement>> uwb_range_window;
    vector<vector<UWBMeasurement>> uwb_range_sol_data;
    ButterworthLowPassFilter *bf;
};
#endif