#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include "estimator/uwb_manager.h"
#include "nlink_parser/LinktrackNodeframe2.h"
UWBManager uwb_manager[20];

std::mutex m_buf;

// 设置随机数生成器
std::default_random_engine generator;
std::normal_distribution<double> noise_normal_distribution(0.0, 0.08);
std::uniform_real_distribution<double> noise_uniform_distribution(-0.1, 0.1);  // 均匀分布
ros::Publisher pub_range_raw;
ros::Publisher pub_range_data;
double anomaly_probability = 0.05;  // 异常值出现的概率
double anomaly_magnitude = 10.0;   // 异常值的大小
double anomaly_window=0.0;
int global_pose=1;


int first_uwb=0;
// Eigen::Vector3d anchor_create_pos[5]={
//     Eigen::Vector3d(-4.17,-4.35,1.38),
//     Eigen::Vector3d(2.93,-3.65,1.3),
//     Eigen::Vector3d(2.76,1.12,1.59),
//     Eigen::Vector3d(-4.48,1.17,1.14)
// };

Eigen::Vector3d anchor_create_pos[5]={
    Eigen::Vector3d(-38.17,-34.35,1.38),
    Eigen::Vector3d(32.93,-36.65,3.3),
    Eigen::Vector3d(38.76,46.12,1.59),
    Eigen::Vector3d(-34.48,31.17,1.14)
};
const int ANCHORNUMBER=4;
ros::Publisher pub_odometry_ran[5];
double getNoiseRandomValue(double dis,Eigen::Vector3d eul)
{
    double noisy_value = noise_normal_distribution(generator)+noise_uniform_distribution(generator)*0.0;
    double anomaly_probability = noise_uniform_distribution(generator);
    // noisy_value=0;
    // 如果满足异常概率，引入异常值
    if(anomaly_window>0.1)
    {
        if(abs(anomaly_probability)*10<anomaly_window){
            double anomaly_magnitude = noise_normal_distribution(generator)*8+noise_uniform_distribution(generator)*8;
            noisy_value += anomaly_magnitude;
        }
        anomaly_window*=0.7;
    }
    else{
        anomaly_window=0.0;
        if (abs(anomaly_probability) < 0.0010) {
        // 异常值突变，大小和概率均随机
            double anomaly_magnitude = noise_normal_distribution(generator)*8+noise_uniform_distribution(generator)*2.5;
            noisy_value += anomaly_magnitude;
            anomaly_window=1.0;
        }
    }
    
    return noisy_value+dis/18;//+(dis/1.8)*0.1;//+abs(eul.x())/180*3.14*0.1+abs(eul.y())/180*3.14*0.08+abs(eul.z())/180*3.14*0.5;
}
void ground_truth_callback(const nav_msgs::OdometryConstPtr &msg,int idx)
{
    m_buf.lock();
    Eigen::Vector3d ps,vs,ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.pose.position, ps);
    tf::vectorMsgToEigen(msg->twist.twist.linear, vs);
    tf::vectorMsgToEigen(msg->twist.twist.angular, ws);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation,rs);
    OdometryVins tmp(ps,vs,ws,rs,msg->header.stamp.toSec());
    nav_msgs::Odometry odo;
    odo=*msg;
    double time=msg->header.stamp.toSec();
    for(int i=0;i<ANCHORNUMBER;i++){
        double range=(ps-anchor_create_pos[i]).norm();
        double range2=range+getNoiseRandomValue(range,Utility::R2ypr(rs.toRotationMatrix()));
        bool res=uwb_manager[idx*(ANCHORNUMBER)+i].addUWBMeasurements(0,time,range2);        
        double dis=uwb_manager[idx*(ANCHORNUMBER)+i].uwb_range_sol_data[0].back().range;
        odo.twist.covariance[i]=dis;
    }
    pub_odometry_ran[idx].publish(odo);
    m_buf.unlock();
}
//60 324 271 974 1000
void ground_truth_callback_2(const geometry_msgs::PoseStampedConstPtr &msg,int idx)
{
    //m_buf.lock();
    Eigen::Vector3d ps,vs,ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.position, ps);
    tf::quaternionMsgToEigen(msg->pose.orientation,rs);
    Eigen::Vector3d camera_ps=ps+rs*Eigen::Vector3d(0.1,0,-0.03);
    OdometryVins tmp(camera_ps,rs,msg->header.stamp.toSec());
    geometry_msgs::PoseStamped tmp2=*msg;
    tf::pointEigenToMsg(camera_ps,tmp2.pose.position);
    nav_msgs::Odometry odo;
    double time=msg->header.stamp.toSec();
    for(int i=0;i<ANCHORNUMBER;i++){
        double range=(ps-anchor_create_pos[i]).norm();
        double range2=range+getNoiseRandomValue(range,Utility::R2ypr(rs.toRotationMatrix()));
        bool res=uwb_manager[idx].addUWBMeasurements(i,time,range2);        
        double dis=uwb_manager[i].uwb_range_sol_data[0].back().range;
        odo.twist.covariance[i]=dis;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Subscriber sub_gt[4];
    int SIM_UE=1;
    if(SIM_UE==1){
        sub_gt[3]=n.subscribe<nav_msgs::Odometry>("/pose_2", 2000, boost::bind(ground_truth_callback, _1, 3));
        sub_gt[2]=n.subscribe<nav_msgs::Odometry>("/pose_3", 2000, boost::bind(ground_truth_callback, _1, 2));
        sub_gt[1]=n.subscribe<nav_msgs::Odometry>("/pose_1", 2000, boost::bind(ground_truth_callback, _1, 1));
    }
    else{
        for(int i=1;i<=3;i++){
            sub_gt[i]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag"
            +to_string(i)+"/pose", 2000, boost::bind(ground_truth_callback_2, _1, i));
        }
        
    }
    for(int i=1;i<=3;i++){
        pub_odometry_ran[i] = n.advertise<nav_msgs::Odometry>("/ag"+to_string(i)+"/vins_estimator/imu_propagate", 1000);
    }
    vector<Eigen::Vector3d>tmp;
    for(int i=0;i<ANCHORNUMBER;i++)tmp.push_back(anchor_create_pos[i]);
    for(int i=0;i<=19;i++){
        uwb_manager[i]=UWBManager(4,tmp);
    }
    ros::spin();

    return 0;
}