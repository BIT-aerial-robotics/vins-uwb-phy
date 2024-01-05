#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/uwb_manager.h"
#include "utility/visualization.h"

const int USE_SIM=0;
const int SOL_LENGTH=100;
const int IMU_PROPAGATE=1;
std::mutex m_buf;
std::map<double,OdometryVins>pose_agent_buf[5];
double para_pos[5][200][3],para_yaw[5][200][1];
void agent_pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg,int id)
{
    m_buf.lock();
    Eigen::Vector3d ws;
    Eigen::Quaterniond qs;
    Eigen::Vector3d ps=Eigen::Vector3d::Zero(),vs=Eigen::Vector3d::Zero();
    tf::quaternionMsgToEigen(pose_msg->pose.pose,qs);
    tf::vectorMsgToEigen(pose_msg->twist.twist.angular,ws);
    tf::vectorMsgToEigen(pose_msg->twist.twist.linear,vs);
    tf::pointMsgToEigen(pose_msg->pose.pose.position,ps);
    double time=imu_msg->header.stamp.toSec();
    pose_agent_buf[id][time]=OdometryVins(ps,vs,ws,qs,time);
    m_buf.unlock();
}
void center_pose_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    m_buf.lock();
    Eigen::Vector3d ws;
    Eigen::Quaterniond qs;
    Eigen::Vector3d ps=Eigen::Vector3d::Zero(),vs=Eigen::Vector3d::Zero();
    tf::vectorMsgToEigen(imu_msg->angular_velocity,ws);
    tf::quaternionMsgToEigen(imu_msg->orientation,qs);
    double time=imu_msg->header.stamp.toSec();
    pose_agent_buf[0][time]=OdometryVins(ps,vs,ws,qs,time);
    m_buf.unlock();
}
void sync_process()
{
    int last_data_size=0;
    int sys_cnt=0;
    int opt_frame_len=0;
    int faile_num=0;
    Eigen::Vector3d ps[5][200],vs[5][200],ws[5][200],qs[5][200],alpha[200];
    while(1)
    {
        TicToc t_sub;
        // ros::Rate loop_rate(200);
        // loop_rate.sleep();
        m_buf.lock();
        int last_opt_frame_len=opt_frame_len;
        while(pose_agent_buf[1].size()>0)
        {
            double time=pose_agent_buf[1].begin()->first;
            OdometryVins ot[4];
            bool ot_flag[4]={false,true,false,false};
            OdometryVins::queryOdometryMap(pose_agent_buf[2],time,ot[2],ot_flag[2]);
            OdometryVins::queryOdometryMap(pose_agent_buf[3],time,ot[3],ot_flag[3]);
            OdometryVins::queryOdometryMap(pose_agent_buf[0],time,ot[0],ot_flag[0]);
            bool flag=ot_flag[0]&ot_flag[1]&ot_flag[2]&ot_flag[3];
            ot[1]=pose_agent_buf[1].begin()->second;
            if(flag==false){
                break;
            }
            else{
                if(opt_frame_len==0){
                    para_pos[1][opt_frame_len][0]=0;    para_pos[1][opt_frame_len][1]=0;    para_pos[1][opt_frame_len][2]=0; para_yaw[1][opt_frame_len][0]= 0;
                    para_pos[2][opt_frame_len][0]=-0.4; para_pos[2][opt_frame_len][1]=-0.7; para_pos[2][opt_frame_len][2]=0; para_yaw[2][opt_frame_len][0]= 120;
                    para_pos[3][opt_frame_len][0]= 0.4; para_pos[3][opt_frame_len][1]=-0.7; para_pos[3][opt_frame_len][2]=0; para_yaw[3][opt_frame_len][0]=-120;
                }
                else{
                    for(int i=1;i<=3;i++){
                        for(int j=0;j<=2;j++)
                        para_pos[i][opt_frame_len][j]=para_pos[i][opt_frame_len-1][j];
                        para_yaw[i][opt_frame_len][0]=para_yaw[i][opt_frame_len-1][0];
                    }
                }
                for(int i=1;i<=3;i++){
                    ps[i][opt_frame_len]=ot[i].Ps;
                    vs[i][opt_frame_len]=ot[i].Vs;
                    ws[i][opt_frame_len]=ot[i].Ws;
                    qs[i][opt_frame_len]=ot[i].Rs;
                }
                alpha[opt_frame_len]=ot[0].Rs.toRotationMatrix()(2,2);
                opt_frame_len++;
                pose_agent_buf[1].erase(pose_agent_buf[1].begin());
                if(opt_frame_len>=SOL_LENGTH)
                break;
            }
        }
        if(opt_frame_len<SOL_LENGTH){
            m_buf.unlock();
            continue;
        }

        sys_cnt+=1;
        
        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        loss_function = new ceres::HuberLoss(1.0);
        for(int i=0;i<opt_frame_len;i++){
            kinFactor_bet_4dof_2 *bxt = new kinFactor_bet_4dof_2(para_Rt[i][k + 1], para_yaw[i][k+1], para_Rt[i][k], para_yaw[i][k], para_agent_time[k] - para_agent_time[k + 1], sigma_rt_6dof_loose);
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<kinFactor_bet_4dof_2, 4, 3, 1,3, 1>(bxt),
                        NULL,
                        para_Rt[i][k + 1], para_yaw[i][k+1], para_Rt[i][k], para_yaw[i][k]);
        }
    }
}
int main()
{
    ros::init(argc, argv, "loose");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::Subscriber sub_agent1_pose, sub_agent2_pose, sub_agent3_pose;
    ros::Subscriber sub_agent0_imu = nh.subscribe("/mavros/imu/data", 2000, center_pose_callback);
    if(IMU_PROPAGATE==1){
        sub_agent1_pose = nh.subscribe("/ag1/vins_estimator/imu_propagate", 2000, boost::bind(agent_pose_callback, _1, 1));
        sub_agent2_pose = nh.subscribe("/ag2/vins_estimator/imu_propagate", 2000, boost::bind(agent_pose_callback, _1, 2));
        sub_agent3_pose = nh.subscribe("/ag3/vins_estimator/imu_propagate", 2000, boost::bind(agent_pose_callback, _1, 3));
    }
    else{
        sub_agent1_pose = nh.subscribe("/ag1/vins_estimator/odometry", 2000, boost::bind(agent_pose_callback, _1, 1));
        sub_agent2_pose = nh.subscribe("/ag2/vins_estimator/odometry", 2000, boost::bind(agent_pose_callback, _1, 2));
        sub_agent3_pose = nh.subscribe("/ag3/vins_estimator/odometry", 2000, boost::bind(agent_pose_callback, _1, 3));
    }
    std::thread sync_thread{sync_process};
    ros::spin();
}