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
OnlineStatistics stats;
UWBManager uwb_manager[5]={
    UWBManager(1,vector<Eigen::Vector3d>(1,Eigen::Vector3d(1,1,1))),
    UWBManager(1,vector<Eigen::Vector3d>(1,Eigen::Vector3d(1,1,1))),
    UWBManager(1,vector<Eigen::Vector3d>(1,Eigen::Vector3d(1,1,1))),
    UWBManager(1,vector<Eigen::Vector3d>(1,Eigen::Vector3d(1,1,1))),
    UWBManager(1,vector<Eigen::Vector3d>(1,Eigen::Vector3d(1,1,1)))
};
int first_uwb=0;
vector<double>range_filter;
void feature_callback(const nlink_parser::LinktrackNodeframe2ConstPtr &msg)
{
    uint32_t num_nodes = msg->nodes.size();
    int id=msg->id;
    nlink_parser::LinktrackNodeframe2 tmp=*msg;
    // 遍历 nodes 数组
    
    for (uint32_t i = 0; i < num_nodes; ++i) {
        // 获取当前节点
        const nlink_parser::LinktrackNode2& node = msg->nodes[i];
        printf("%d %lf  ",node.id,node.dis);
        stats.update(node.dis*100);
        double time;
        if(first_uwb==0)
        {
            time=1703644124.77;
            first_uwb=msg->system_time;
        }
        else
        {
            time=1703644124.77+(msg->system_time-first_uwb)*1.00/1000.00;
        }
        bool flag = uwb_manager[id].addUWBMeasurements(0,time,node.dis);
        if(flag)
        {
            double dis=uwb_manager[id].uwb_range_sol_data[0].back().range;
            range_filter.push_back(dis);
            tmp.nodes[i].dis=dis;
            pub_feature.publish(tmp);
        }
        // if(range_filter.size()%10==1&&range_filter.size()>100){
        //     shapiro_wilk_test(range_filter.begin(), range_filter.end(), &w, &p_value);
        // }
        std::cout << "Mean: " << stats.mean() << ", Variance: " << stats.variance() <<" "<<p_value << std::endl;
        // 在这里使用 node 进行操作 
        // 例如：node.role, node.id, node.local_time, 等等
    }printf("\n");
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::Subscriber sub_feature = n.subscribe("/nlink_linktrack_nodeframe2", 2000, feature_callback);
    pub_feature = n.advertise<nlink_parser::LinktrackNodeframe2>("/nlink_sol",1000);
    ros::spin();
    return 0;
}