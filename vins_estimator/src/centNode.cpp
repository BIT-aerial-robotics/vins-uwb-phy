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

map<double, OdometryVins> data[5];
ros::Publisher pub_cent_odometry;
std::mutex m_buf;
int USE_GT=0;
std::default_random_engine generator;
std::normal_distribution<double> noise_normal_distribution(0.0, 0.08);
void self_odometry_callback(const nav_msgs::OdometryConstPtr &msg, int idx)
{
    m_buf.lock();
    Eigen::Vector3d ps, vs, ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.pose.position, ps);
    
    tf::vectorMsgToEigen(msg->twist.twist.linear, vs);
    tf::vectorMsgToEigen(msg->twist.twist.angular, ws);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation, rs);
    OdometryVins tmp(ps, vs, ws, rs, msg->header.stamp.toSec());
    data[idx][tmp.time]=tmp;
    m_buf.unlock();
}
void self_odometry_callback_2(const geometry_msgs::PoseStampedConstPtr &msg, int idx)
{
    m_buf.lock();
    Eigen::Vector3d ps, vs, ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.position, ps);
    tf::quaternionMsgToEigen(msg->pose.orientation, rs);
    OdometryVins tmp(ps,rs, msg->header.stamp.toSec());
    data[idx][tmp.time]=tmp;
    m_buf.unlock();
}
void sync_process()
{
    int failnum=0;
    while (1)
    {
        double time = 0;
        m_buf.lock();
        //cout<<data[1].size()<<" "<<data[2].size()<<" "<<data[3].size()<<" "<<failnum<<endl;
        if (!data[1].empty())
        {
            bool f2=false,f3=false;
            OdometryVins a_now[4];
            a_now[1]=data[1].begin()->second;
            f2=OdometryVins::queryOdometryMap(data[2],a_now[1].time,a_now[2],0.02);
            f3=OdometryVins::queryOdometryMap(data[3],a_now[1].time,a_now[3],0.02);
            // if(abs(noise_normal_distribution(generator))>0.16)
            // {
            //     for(int i=1;i<=3;i++)
            //     a_now[i].Ps+=Eigen::Vector3d(noise_normal_distribution(generator),noise_normal_distribution(generator),noise_normal_distribution(generator));
            // }
            if (f2&&f3)
            {
                Eigen::Vector3d ps = Eigen::Vector3d::Zero(), vs = Eigen::Vector3d::Zero(), ws;
                for (int i = 1; i <= 3; i++)
                {
                    if(USE_GT==0){
                        ps += a_now[i].Ps;
                        vs += a_now[i].Vs;
                    }
                    else{
                        ps += a_now[i].Ps+a_now[i].Rs*Eigen::Vector3d(-0.0,0.00,-0.03);
                        vs += a_now[i].Vs;
                    }
                    
                }
                ps /= 3;
                vs /= 3;
                // Eigen::Vector3d head_dir;
                // head_dir=a_now[1].Ps-ps;
                // head_dir(2)=0.0;
                // double head_2d_norm = head_dir.norm();
                // double cos_yaw = head_dir(0)/head_2d_norm;
                // double sin_yaw = head_dir(1)/head_2d_norm;
                // Eigen::Matrix3d R_yaw;
                // R_yaw.setIdentity();
                // R_yaw(0,0)=cos_yaw;R_yaw(0,1)=-sin_yaw;
                // R_yaw(1,0)=sin_yaw;R_yaw(1,1)=-cos_yaw;
                Eigen::Vector3d p1 = a_now[1].Ps - (a_now[2].Ps*0.5+a_now[3].Ps*0.5), p2 = a_now[2].Ps - a_now[3].Ps;
                p1.normalize();
                p2.normalize();
                Eigen::Vector3d p3 = p1.cross(p2);
                p3.normalize();
                Eigen::Matrix3d rot;
                rot(0, 0) = p1(0), rot(0, 1) = p1(1), rot(0, 2) = p1(2);
                rot(1, 0) = p2(0), rot(1, 1) = p2(1), rot(1, 2) = p2(2);
                rot(2, 0) = p3(0), rot(2, 1) = p3(1), rot(2, 2) = p3(2);
                rot.transposeInPlace();
                OdometryVins cent(ps, vs, ws, Eigen::Quaterniond{rot}, a_now[1].time);
                nav_msgs::Odometry odometry;
                odometry.header.stamp = ros::Time(a_now[1].time);
                odometry.header.frame_id = "world";
                tf::pointEigenToMsg(ps, odometry.pose.pose.position);
                tf::quaternionEigenToMsg(cent.Rs, odometry.pose.pose.orientation);
                tf::vectorEigenToMsg(cent.Vs, odometry.twist.twist.linear);
                pub_cent_odometry.publish(odometry);
                failnum=0;
                data[1].erase(data[1].begin());
            }
            else{
                failnum++;
            }
        }
        if(failnum>=40){
            failnum=0;
            data[1].erase(data[1].begin());
        }
        while(data[2].size()>0&&data[2].begin()->second.time<data[1].begin()->second.time-0.1)
        data[2].erase(data[2].begin());
        while(data[3].size()>0&&data[3].begin()->second.time<data[1].begin()->second.time-0.1)
        data[3].erase(data[3].begin());
        m_buf.unlock();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::Subscriber sub_self_odometry[4];
    pub_cent_odometry = n.advertise<nav_msgs::Odometry>("/ag0/vins_estimator/imu_propagate", 1000);
    for (int i = 1; i <= 3; i++)
    {
        //calib_pose
        if(USE_GT==0)
        //sub_self_odometry[i] = n.subscribe<nav_msgs::Odometry>("/ag" + std::to_string(i) + "/calib_pose", 500, boost::bind(self_odometry_callback, _1, i));
        sub_self_odometry[i] = n.subscribe<nav_msgs::Odometry>("/ag" + std::to_string(i) + "/vins_estimator/imu_propagate", 500, boost::bind(self_odometry_callback, _1, i));
        else
        sub_self_odometry[i] = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag" + std::to_string(i) + "/pose", 500, boost::bind(self_odometry_callback_2, _1, i));
    }
    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}