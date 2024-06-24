#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include "estimator/uwb_manager.h"
#include "nlink_parser/LinktrackNodeframe2.h"
#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions.hpp>
boost::math::normal_distribution<double> normal_dist;
double w, p_value;
std::mutex m_buf;
ros::Publisher  pub_feature;
OnlineStatistics stats[10];
UWBManager uwb_manager[5]={
    UWBManager(1,vector<Eigen::Vector3d>(1,Eigen::Vector3d(1,1,1))),
    UWBManager(1,vector<Eigen::Vector3d>(1,Eigen::Vector3d(1,1,1))),
    UWBManager(1,vector<Eigen::Vector3d>(1,Eigen::Vector3d(1,1,1))),
    UWBManager(1,vector<Eigen::Vector3d>(1,Eigen::Vector3d(1,1,1))),
    UWBManager(1,vector<Eigen::Vector3d>(1,Eigen::Vector3d(1,1,1)))
};
int first_uwb=0;
vector<double>range_filter;
int nxt_idx[15]={
    0,1,2,3,0,1,2,3,4,4
};
void feature_callback(const nlink_parser::LinktrackNodeframe2ConstPtr &msg)
{
    uint32_t num_nodes = msg->nodes.size();
    int id2=msg->id;
    nav_msgs::Odometry tmp;
    // 遍历 nodes 数组
    vector<int >ids;
    vector<double> dis_tag;
    for (uint32_t i = 0; i < num_nodes; ++i) {
        // 获取当前节点
        const nlink_parser::LinktrackNode2& node = msg->nodes[i];
        double dis=node.dis;
        double time=msg->header.stamp.toSec();
        int id=node.id;

        bool res=uwb_manager[nxt_idx[id]].addUWBMeasurements(0,time,node.dis);
        dis=uwb_manager[nxt_idx[id]].uwb_range_sol_data[0].back().range;
        tmp.pose.covariance[id-4]=dis;
        ids.push_back(node.id-4);
        if(res){
            stats[id-4].update(dis);
        }
        dis_tag.push_back(dis);
        // 在这里使用 node 进行操作 
        // 例如：node.role, node.id, node.local_time, 等等
    }
    for(int i=0;i<ids.size();i++){
        printf("(%d %lf %lf %lf) ",ids[i],dis_tag[i],stats[ids[i]].mean()*0.97-0.44,stats[ids[i]].variance());
    }printf("\n");
    pub_feature.publish(tmp);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::Subscriber sub_feature = n.subscribe("/u0/nlink_linktrack_nodeframe2", 2000, feature_callback);
    pub_feature = n.advertise<nav_msgs::Odometry>("/nlink_sol",1000);
    ros::spin();
    return 0;
}