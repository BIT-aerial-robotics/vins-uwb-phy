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

int USE_FLY=0;
Estimator estimator;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
UWBManager uwb_manager[5];
std::mutex m_buf;

// 设置随机数生成器
std::default_random_engine generator;
std::normal_distribution<double> noise_normal_distribution(0.0, 0.09);
std::uniform_real_distribution<double> noise_uniform_distribution(-0.1, 0.1);  // 均匀分布
ros::Publisher pub_range_raw;
ros::Publisher pub_range_data;
double anomaly_probability = 0.05;  // 异常值出现的概率
double anomaly_magnitude = 10.0;   // 异常值的大小
double anomaly_window=0.0;
ros::Publisher pub_est_mark;
int global_pose=1;
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    // Vector3d acc(dx, dy, dz);
    // Vector3d gyr(rx, ry, rz);
    //# 右下前转前左上
    if(SIM_UE==1){
        Vector3d acc(dx, dy, dz);
        Vector3d gyr(rx, ry, rz);
        estimator.inputIMU(t, acc, gyr);
    }
    else{
        Vector3d acc(dz, -dx, -dy);
        Vector3d gyr(rz, -rx, -ry);
        estimator.inputIMU(t, acc, gyr);
    }
    
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}



int first_uwb=0;
Eigen::Vector3d anchor_create_pos[5]={
    Eigen::Vector3d(-4.17,-4.35,1.38),
    Eigen::Vector3d(2.93,-3.65,1.3),
    Eigen::Vector3d(2.76,1.12,1.59),
    Eigen::Vector3d(-4.48,1.17,1.14)
};

// Eigen::Vector3d anchor_create_pos[6]={
//     Eigen::Vector3d(-38.17,-34.35,1.38),
//     Eigen::Vector3d(32.93,-36.65,3.3),
//     Eigen::Vector3d(38.76,46.12,1.59),
//     Eigen::Vector3d(-34.48,31.17,1.14),
//     Eigen::Vector3d(-4.48,1.17,3.14)
// };
//const int ANCHORNUMBER=5;
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
    
    return noisy_value;//+abs(eul.x())/180*3.14*0.1+abs(eul.y())/180*3.14*0.08+abs(eul.z())/180*3.14*0.5;
}

nav_msgs::Path path_cent;
ros::Publisher pub_path_cent;
void ground_truth_callback(const nav_msgs::OdometryConstPtr &msg,int idx)
{
    if(idx==0&&AGENT_NUMBER==1){
        if(estimator.to_world_rt_flag){
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = msg->header;
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose = msg->pose.pose;
            path_cent.header = msg->header;
            path_cent.header.frame_id = "world";
            path_cent.poses.push_back(pose_stamped);
            pub_path_cent.publish(path_cent);
        }
    }
    if(idx!=AGENT_NUMBER)return;
    m_buf.lock();
    Eigen::Vector3d ps,vs,ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.pose.position, ps);
    tf::vectorMsgToEigen(msg->twist.twist.linear, vs);
    tf::vectorMsgToEigen(msg->twist.twist.angular, ws);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation,rs);
    OdometryVins tmp(ps,vs,ws,rs,msg->header.stamp.toSec());
    estimator.inputGT(idx,tmp);
    if(idx==AGENT_NUMBER)
    {
        double time=msg->header.stamp.toSec();
        geometry_msgs::PoseArray raw,data;
        raw.header=msg->header;
        data.header=msg->header;
        // //nlink_parser::LinktrackNodeframe2 frame;
        for(int i=0;i<ANCHORNUMBER;i++){
            double range=(ps-anchor_create_pos[i]).norm();
            //ROS_INFO("%lf %lf %lf",anchor_create_pos[i](0),anchor_create_pos[i](1),anchor_create_pos[i](2));
            double range2=range+getNoiseRandomValue(range,Utility::R2ypr(rs.toRotationMatrix()));
            bool res=uwb_manager[i].addUWBMeasurements(0,time,range2);
            geometry_msgs::Pose raw_pose;
            raw.poses.push_back(raw_pose);
            if(res){
                double dis=uwb_manager[i].uwb_range_sol_data[0].back().range;
                //estimator.inputRange(i,time,dis);
                //ROS_INFO("%d %d %d %lf %lf",AGENT_NUMBER,idx,i,time,dis);
                //geometry_msgs::Pose data_pose;
                //data_pose.position.x=dis;
                //data.poses.push_back(data_pose);
            }
            else{
                //geometry_msgs::Pose data_pose;
                //data_pose.position.x=uwb_manager[i].uwb_range_sol_data[0].back().range;
                //data.poses.push_back(data_pose);
            }
        }
        //pub_range_raw.publish(raw);
        //pub_range_data.publish(data);
    }
    
    //nav_msgs::OdometryConstPtr odomPtr = boost::make_shared<const nav_msgs::Odometry>(odom);
    m_buf.unlock();
}

void ground_truth_callback_2(const geometry_msgs::PoseStampedConstPtr &msg,int idx)
{
    if(idx!=AGENT_NUMBER)return;
    //m_buf.lock();
    Eigen::Vector3d ps,vs,ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.position, ps);
    tf::quaternionMsgToEigen(msg->pose.orientation,rs);
    Eigen::Vector3d camera_ps=ps+rs*Eigen::Vector3d(0.1,0,-0.03);
    OdometryVins tmp(camera_ps,rs,msg->header.stamp.toSec());
    if(idx==AGENT_NUMBER)estimator.inputGT(idx,tmp);
    geometry_msgs::PoseStamped tmp2=*msg;
    tf::pointEigenToMsg(camera_ps,tmp2.pose.position);
    pub_est_mark.publish(tmp2);
    if(SIM_UWB)
    {
        double time=msg->header.stamp.toSec();
        geometry_msgs::PoseArray raw,data;
        raw.header=msg->header;
        data.header=msg->header;
        for(int i=0;i<ANCHORNUMBER;i++){
            double range=(ps-anchor_create_pos[i]).norm();
            double range2=range+getNoiseRandomValue(range,Utility::R2ypr(rs.toRotationMatrix()));
            bool res=uwb_manager[i].addUWBMeasurements(0,time,range2);
            geometry_msgs::Pose raw_pose;
            raw_pose.position.x=range2;
            raw.poses.push_back(raw_pose);
            double dis=uwb_manager[i].uwb_range_sol_data[0].back().range;
            raw_pose.position.y=abs(range-dis);
            geometry_msgs::Pose data_pose;
            data_pose.position.x=dis;
            data.poses.push_back(data_pose);
            if(res){
               //estimator.inputRange(i,time,dis);
               
            }
            else{
                
            }
        }
        pub_range_raw.publish(raw);
        pub_range_data.publish(data);
    }
    if(global_pose)
    {
        OdometryVins tmp(ps,vs,ws,rs,msg->header.stamp.toSec());
        estimator.inputOtherPose(idx,tmp);
    }
}
void anchor_call_back(const nav_msgs::OdometryConstPtr &msg,int idx)
{
    //m_buf.lock();
    printf("anchor_call_back\n");
    Eigen::Vector3d ps,vs,ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.pose.position, ps);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation,rs);
    double beta=msg->twist.covariance[(AGENT_NUMBER-1)*2+1];
    double gamma=msg->twist.covariance[(AGENT_NUMBER-1)*2+0];
    OdometryVins tmp(ps,rs,msg->header.stamp.toSec());
    estimator.inputAnchor(idx,tmp,beta,gamma);
    //m_buf.unlock();
}
void rt_call_back(const geometry_msgs::PoseStampedConstPtr &msg)
{
    //m_buf.lock();
    printf("rt _ call _ back\n");
    Eigen::Vector3d ps,vs,ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.position, ps);
    tf::quaternionMsgToEigen(msg->pose.orientation,rs);
    OdometryVins tmp(ps,rs,msg->header.stamp.toSec());
    estimator.inputRt(tmp);
    //m_buf.unlock();
}
int idx_2_idx[]={3,8,1,2};
void uwb_callback(const nlink_parser::LinktrackNodeframe2ConstPtr &msg)
{
    if(USE_FLY)
    {
        if(UWB_TAG_ID!=(int)(msg->id))return;
    }
    else{
        if(idx_2_idx[AGENT_NUMBER]!=(int)(msg->id))return;
    }
    uint32_t num_nodes = msg->nodes.size();
    nlink_parser::LinktrackNodeframe2 tmp=*msg;
    //遍历 nodes 数组
    geometry_msgs::PoseArray raw,data;
    data.header=msg->header;
    for (uint32_t i = 0; i < num_nodes; ++i) {
        //获取当前节点
        const nlink_parser::LinktrackNode2& node = msg->nodes[i];
        double time=msg->header.stamp.toSec();
        int id=(int)(node.id);
        if(id<4)continue;
        if(id==9)id=7;
        bool res=uwb_manager[id-4].addUWBMeasurements(0,time,node.dis);
        double dis=uwb_manager[id-4].uwb_range_sol_data[0].back().range;
        if(res){
            estimator.inputRange(id-4,time,dis);
        }
        else{
            
        }
    }
    //pub_range_data.publish(data);
    //m_buf.unlock();
    //printf("\n");
}
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

void self_odometry_callback(const nav_msgs::OdometryConstPtr &msg,int idx)
{
    //m_buf.lock();

    Eigen::Vector3d ps,vs,ws;
    Eigen::Quaterniond rs;
    tf::pointMsgToEigen(msg->pose.pose.position,ps);
    tf::vectorMsgToEigen(msg->twist.twist.linear,vs);
    tf::vectorMsgToEigen(msg->twist.twist.angular,ws);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation,rs);
    if(idx==AGENT_NUMBER)
    {
        for(int i=0;i<ANCHORNUMBER;i++){
            //uwb_manager[i].addOdometryMeasurements(ps,vs,ws,rs,msg->header.stamp.toSec());
        }
    }
    if(USE_LOOSE==0)
    {
        OdometryVins tmp(ps,vs,ws,rs,msg->header.stamp.toSec());
        estimator.inputOtherPose(idx,tmp);
    }
    else{
        ps=ps+rs*HINGE;
        OdometryVins tmp(ps,vs,ws,rs,msg->header.stamp.toSec());
        estimator.inputOtherPose(idx,tmp);
    }
    
    //m_buf.unlock();
}

double z_val_min=1,z_val_max=-1;
void center_imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    Quaterniond q;
    tf::quaternionMsgToEigen(imu_msg->orientation,q);
    Eigen::Matrix3d rot=q.toRotationMatrix();
    double z_val=rot(2,2);
    z_val_min=min(z_val_min,z_val);
    z_val_max=max(z_val_max,z_val);
    OdometryVins tmp(Vector3d::Zero(),acc,gyr,q,imu_msg->header.stamp.toSec());
    estimator.inputOtherPose(0,tmp);
    return;
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);
    if (n.getParam("/real_fly", USE_FLY))
    {
        ROS_INFO("rosNodeTest real fly: %d\n", USE_FLY);
    }
    else{
        USE_FLY=0;
        ROS_INFO("rosNodeTest real fly moren config : %d\n", USE_FLY);
    }
    ros::Subscriber sub_imu;
    if(USE_IMU)
    {
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    }
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1;
    if(STEREO)
    {
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    }
    ros::Subscriber sub_restart = n.subscribe("vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("vins_cam_switch", 100, cam_switch_callback);
    ros::Subscriber sub_uwb_range[4];
    ros::Subscriber sub_uwb_anchor[4];
    pub_est_mark=n.advertise<geometry_msgs::PoseStamped>("est_mark", 1000);
    for(int i=0;i<=3;i++){
        if(SIM_UWB==0)
            sub_uwb_range[i]=n.subscribe<nlink_parser::LinktrackNodeframe2>("/u"+std::to_string(i)+"/nlink_linktrack_nodeframe2", 500, uwb_callback);
    }
    pub_range_raw=n.advertise<geometry_msgs::PoseArray>("range_raw", 1000);
    pub_range_data=n.advertise<geometry_msgs::PoseArray>("range_sol", 1000);
    ros::Subscriber sub_gt[4];
    if(SIM_UE==1){
        sub_gt[3]=n.subscribe<nav_msgs::Odometry>("/pose_2", 2000, boost::bind(ground_truth_callback, _1, 3));
        sub_gt[2]=n.subscribe<nav_msgs::Odometry>("/pose_3", 2000, boost::bind(ground_truth_callback, _1, 2));
        sub_gt[1]=n.subscribe<nav_msgs::Odometry>("/pose_1", 2000, boost::bind(ground_truth_callback, _1, 1));
        for(int i=0;i<4;i++){
            for(int j=0;j<=1;j++)
            {
                //anchor_create_pos[i](j)=anchor_create_pos[i](j)*2;
            }
        }

        if(AGENT_NUMBER==1){
            sub_gt[0]=n.subscribe<nav_msgs::Odometry>("/pose_0", 2000, boost::bind(ground_truth_callback, _1, 0));
            pub_path_cent=n.advertise<nav_msgs::Path>("/ag0/vins_estimator/path", 1000);
        }   
    }
    else{
        //sub_gt[3]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag3/pose", 2000, boost::bind(ground_truth_callback_2, _1, 3));
        //sub_gt[2]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag2/pose", 2000, boost::bind(ground_truth_callback_2, _1, 2));
        //sub_gt[1]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag1/pose", 2000, boost::bind(ground_truth_callback_2, _1, 1));
        //sub_gt=n.subscribe("/vrpn_client_node/robot"+std::to_string(AGENT_NUMBER)+"2/pose", 2000, ground_truth_callback2);
        sub_gt[AGENT_NUMBER]=n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/ag"+to_string(AGENT_NUMBER)+"/pose", 2000, boost::bind(ground_truth_callback_2, _1, AGENT_NUMBER));
    }
    for(int i=0;i<=4;i++){
        uwb_manager[i]=UWBManager(1,vector<Eigen::Vector3d>(1,anchor_create_pos[i]));
    }
    ros::Subscriber sub_self_odometry[4];
    for(int i=1;i<=3;i++){
        if(USE_LOOSE==0)
        {
            sub_self_odometry[i]=n.subscribe<nav_msgs::Odometry>("/ag"+std::to_string(i)+"/vins_estimator/imu_propagate", 
            500, boost::bind(self_odometry_callback, _1, i));
        }
        
        else{
            sub_self_odometry[i]=n.subscribe<nav_msgs::Odometry>("/vins_" + to_string(i)+"/world_pose", 
            500, boost::bind(self_odometry_callback, _1, i));
        }
    }
    sub_self_odometry[0]=n.subscribe<sensor_msgs::Imu>("/mavros/imu/data",200,center_imu_callback);
    ros::Subscriber sub_uwb_anchor_pose[5],self_rt_world;
    if(USE_EST_UWB)
    {
        for(int i=0;i<ANCHORNUMBER;i++){
            sub_uwb_anchor_pose[i]=n.subscribe<nav_msgs::Odometry>("/anchor_pos"+std::to_string(i),500,boost::bind(anchor_call_back,_1,i));
        }
        self_rt_world=n.subscribe<geometry_msgs::PoseStamped>("/ag"+std::to_string(AGENT_NUMBER)+"/rt_world",500,rt_call_back);
    }  
    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}