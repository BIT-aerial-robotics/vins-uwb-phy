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
#include <fstream>


std::mutex m_buf;
UWBManager uwb_manager[50];
// 设置随机数生成器
std::default_random_engine generator;
std::normal_distribution<double> noise_normal_distribution(0.0, 0.09);
std::uniform_real_distribution<double> noise_uniform_distribution(-0.1, 0.1);  // 均匀分布
std::map<double,OdometryVins>buf[5];
double anomaly_probability = 0.05;  // 异常值出现的概率
double anomaly_magnitude = 10.0;   // 异常值的大小
double anomaly_window=0.0;
ros::Publisher pub_est_mark;
int global_pose=1;
double getNoiseRandomValue(double dis,Eigen::Vector3d eul)
{
    double noisy_value = noise_normal_distribution(generator)+noise_uniform_distribution(generator)*0.0;
    double anomaly_probability = noise_uniform_distribution(generator);
    //noisy_value=0;
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
    
    return noisy_value;//+(dis/1.8)*0.1;//+abs(eul.x())/180*3.14*0.1+abs(eul.y())/180*3.14*0.08+abs(eul.z())/180*3.14*0.5;
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
    //nav_msgs::OdometryConstPtr odomPtr = boost::make_shared<const nav_msgs::Odometry>(odom);
    m_buf.unlock();
}

void ground_truth_callback_2(const geometry_msgs::PoseStampedConstPtr &msg,int idx)
{
    //m_buf.lock();
    Eigen::Vector3d ps,vs,ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.position, ps);
    tf::quaternionMsgToEigen(msg->pose.orientation,rs);
    Eigen::Vector3d uwb_ps=ps+rs*Eigen::Vector3d(0,0,-0.03);
    OdometryVins tmp(uwb_ps,rs,msg->header.stamp.toSec());
    double time=msg->header.stamp.toSec();
    buf[idx][time]=tmp;
    //printf("%lf %d\n",time,buf[idx].size());
    
}
void anchor_call_back(const geometry_msgs::PoseStampedConstPtr &msg,int idx)
{
    //m_buf.lock();
    Eigen::Vector3d ps,vs,ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.position, ps);
    tf::quaternionMsgToEigen(msg->pose.orientation,rs);
    OdometryVins tmp(ps,rs,msg->header.stamp.toSec());
    //m_buf.unlock();
}

int uav_idx_2_uwb_idx[]={3,8,1,2};
int uwb_idx_2_uav_idx[]={0,2,3,0,4,5,6,7,1};
Eigen::Vector3d anchor_create_pos[5]={
    Eigen::Vector3d(-4.17,-4.35,1.38),
    Eigen::Vector3d(2.93,-3.65,1.3),
    Eigen::Vector3d(2.76,1.12,1.59),
    Eigen::Vector3d(-4.48,1.17,1.14)
};

void uwb_callback(const nlink_parser::LinktrackNodeframe2ConstPtr &msg)
{
    //printf("%d %d\n",idx_2_idx[AGENT_NUMBER],msg->id);
    //if(idx_2_idx[AGENT_NUMBER]!=(int)(msg->id))return;
    uint32_t num_nodes = msg->nodes.size();
    nlink_parser::LinktrackNodeframe2 tmp=*msg;
    //遍历 nodes 数组
    geometry_msgs::PoseArray raw,data;
    int uav_id=uwb_idx_2_uav_idx[(int)msg->id];
    data.header=msg->header;
    //printf("\n %lf %d \n",msg->header.stamp.toSec(),(int)(msg->id));
    string filename = "/home/f404/output.csv";
    ofstream file(filename,ios::app);
    for (uint32_t i = 0; i < num_nodes; ++i) {
        //获取当前节点
        const nlink_parser::LinktrackNode2& node = msg->nodes[i];
        double time=msg->header.stamp.toSec();
        double dis=node.dis;
        int id=(int)(node.id);
        if(id<4||id>7)continue;

        bool res=uwb_manager[id-4].addUWBMeasurements(0,time,dis);
        dis=uwb_manager[id-4].uwb_range_sol_data[0].back().range;
        if(res){
            OdometryVins query;
            
            bool f=OdometryVins::queryOdometryMap(buf[uav_id],time,query,0.15);
            if(buf[uav_id].size()>0)
            printf("%d %d %d %lf %lf %lf %d\n",(int)msg->id,uav_id,buf[uav_id].size(),time,
            buf[uav_id].begin()->first,buf[uav_id].rbegin()->first,f);
            if(f){
                string con=to_string(time)+","+to_string(anchor_create_pos[id-4](0))+","+to_string(anchor_create_pos[id-4](1))+","+
                to_string(anchor_create_pos[id-4](2))+",0,0,0,";
                Eigen::Vector3d euler=Utility::R2ypr(query.Rs.toRotationMatrix());
                Eigen::Vector3d ps=query.Ps;
                Eigen:: Vector3d dt=ps-anchor_create_pos[id-4];
                std::cout<<ps<<std::endl;
                con=con+to_string(ps(0))+","+to_string(ps(1))+","+to_string(ps(2))+","+to_string(euler(0))+","+to_string(euler(1))+","
                +to_string(euler(2))+","+to_string(dis)+","+to_string(dt.norm());
                file<<con+"\n";
            }
        }
    }file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n;
    ros::Subscriber sub_uwb_range[4];
    for(int i=0;i<=3;i++){
        sub_uwb_range[i]=n.subscribe<nlink_parser::LinktrackNodeframe2>("/u"+std::to_string(i)+"/nlink_linktrack_nodeframe2", 500, uwb_callback);
        //sub_uwb_anchor[i]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/anchor_Marker"+std::to_string(i+1)+"/pose", 50, 
        //boost::bind(anchor_call_back,_1,i));
    }
    ros::Subscriber sub_gt[4];
    if(0){
        sub_gt[3]=n.subscribe<nav_msgs::Odometry>("/pose_2", 2000, boost::bind(ground_truth_callback, _1, 3));
        sub_gt[2]=n.subscribe<nav_msgs::Odometry>("/pose_3", 2000, boost::bind(ground_truth_callback, _1, 2));
        sub_gt[1]=n.subscribe<nav_msgs::Odometry>("/pose_1", 2000, boost::bind(ground_truth_callback, _1, 1));
    }
    else{
        sub_gt[3]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag3/pose", 2000, boost::bind(ground_truth_callback_2, _1, 3));
        sub_gt[2]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag2/pose", 2000, boost::bind(ground_truth_callback_2, _1, 2));
        sub_gt[1]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag1/pose", 2000, boost::bind(ground_truth_callback_2, _1, 1));
        sub_gt[0]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag0/pose", 2000, boost::bind(ground_truth_callback_2, _1, 0));
        //sub_gt=n.subscribe("/vrpn_client_node/robot"+std::to_string(AGENT_NUMBER)+"2/pose", 2000, ground_truth_callback2);
        //sub_gt[AGENT_NUMBER]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag"+to_string(AGENT_NUMBER)+"/pose", 2000, boost::bind(ground_truth_callback_2, _1, AGENT_NUMBER));
    }
    for(int i=0;i<=49;i++){
        uwb_manager[i]=UWBManager(1,vector<Eigen::Vector3d>(1,anchor_create_pos[0]));
    }
    ros::spin();

    return 0;
}