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

double para_HINGE[1] = {-0.09};
double para_LENGTH[1] = {0.841};
double pre_calc_hinge[1] = {para_HINGE[0]};
double pre_calc_length[1] = {para_LENGTH[0]};
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

void pub(int tot)
{

    for (int i = 0; i < tot; i++)
    {
        std_msgs::Header head = data[1].lower_bound(para_agent_time[i])->second.H;
        // std::cout<<head.stamp<<" ";
        for (int j = 1; j <= 3; j++)
        {
            Eigen::Quaterniond lr(Utility::ypr2R(Eigen::Vector3d(para_yaw[j][i][0], 0, 0)));
            lr.normalize();
            Eigen::Vector3d pos(para_Rt[j][i][0], para_Rt[j][i][1], para_Rt[j][i][2]);
            RT[j][para_agent_time[i]] = RTType2(pos, lr, para_yaw[j][i][0]);

            // nav_msgs::Odometry odometry;
            // odometry.header = head;
            // odometry.header.frame_id = "world";
            // odometry.child_frame_id = "world";
            // tf::pointEigenToMsg(pos, odometry.pose.pose.position);
            // tf::quaternionEigenToMsg(lr, odometry.pose.pose.orientation);
            // pub_odometry_frame[j].publish(odometry);
        }
    }

    // std::cout<<"publish rt_world finish"<<endl;
    for (int i = tot - 1; i >= 0; i--)
    {
        std_msgs::Header head = data[1].lower_bound(para_agent_time[i])->second.H;
        if (isPub[para_agent_time[i]] == 0)
        {
            for (int j = 1; j <= 3; j++)
            {
                Eigen::Quaterniond lr(Utility::ypr2R(Eigen::Vector3d(para_yaw[j][i][0], 0, 0)));
                lr.normalize();
                Eigen::Vector3d pos(para_Rt[j][i][0], para_Rt[j][i][1], para_Rt[j][i][2]);
                Eigen::Vector3d a_pos(para_local_pose[j][i][0], para_local_pose[j][i][1], para_local_pose[j][i][2]);
                Eigen::Quaterniond a_r(para_local_pose[j][i][6], para_local_pose[j][i][3], para_local_pose[j][i][4], para_local_pose[j][i][5]);


                if(a_pos.norm()>100 || pos.norm() >100){
                    ROS_DEBUG("ttttttttttttttttttttttttttt %lf  %lf",a_pos.norm(),pos.norm());
                }
                a_r = lr * a_r;
                a_r.normalize();

                a_pos = lr * a_pos + pos;

                nav_msgs::Odometry odometry2;
                odometry2.header = head;
                odometry2.header.frame_id = "world";
                odometry2.child_frame_id = "world";
                tf::pointEigenToMsg(pos, odometry2.pose.pose.position);
                tf::quaternionEigenToMsg(lr, odometry2.pose.pose.orientation);
                pub_odometry_frame[j].publish(odometry2);



                nav_msgs::Odometry odometry;
                odometry.header = head;
                odometry.header.frame_id = "world";
                odometry.child_frame_id = "world";
                tf::pointEigenToMsg(a_pos, odometry.pose.pose.position);
                tf::quaternionEigenToMsg(a_r, odometry.pose.pose.orientation);

                pub_odometry_value[j].publish(odometry);
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header = head;
                pose_stamped.header.frame_id = "world";

                tf::pointEigenToMsg(a_pos, pose_stamped.pose.position);
                tf::quaternionEigenToMsg(a_r, pose_stamped.pose.orientation);

                path_o[j].header = head;
                path_o[j].header.frame_id = "world";
                path_o[j].poses.push_back(pose_stamped);
                pub_path[j].publish(path_o[j]);
            }
            isPub[para_agent_time[i]] = 1;
        }
    }
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
        for(int i=1;i<=3;i++){

            for(int k=1;k<opt_frame_len;k++){
                kinFactor_bet_4dof_2 *bxt = new kinFactor_bet_4dof_2(para_Rt[i][k], para_yaw[i][k], para_Rt[i][k-1], para_yaw[i][k-1], para_agent_time[k+1] - para_agent_time[k], sigma_rt_6dof_loose);
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<kinFactor_bet_4dof_2, 4, 3, 1,3, 1>(bxt),
                    NULL,
                    para_Rt[i][k], para_yaw[i][k], para_Rt[i][k-1], para_yaw[i][k-1]);
            }
        }
        for(int i=1;i<=3;i++){

            for(int k=0;k<opt_frame_len;k++){
                kinFactor_bet_old_4dof_2 *bxt = new kinFactor_bet_old_4dof_2(para_Rt[i][k], para_yaw[i][k], sigma_vins_6dof_loose);
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<kinFactor_bet_old_4dof_2, 4, 3, 1>(bxt),
                    NULL,
                    para_Rt[i][k], para_yaw[i][k]);
            }
        }

        for (int i = 1; i <= 3; i++)
        {
            for (int j = i + 1; j <= 3; j++)
            {
                for (int k = 0; k < opt_frame_len; k++)
                {
                    // int Two = 1;
                    // if (LINER == 1 && i != 2 && abs(i - j) != 1)
                    // {
                    //     Two = 2;
                    // }
                    kinFactor_connect_4dof *self_factor = new kinFactor_connect_4dof(para_local_pose[i][k], para_local_pose[j][k], pre_calc_hinge[0], sigma_length_loose);
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<kinFactor_connect_4dof, 1, 3, 3, 1, 1, 1>(self_factor),
                        NULL,
                        para_Rt[i][k], para_Rt[j][k], para_yaw[i][k], para_yaw[j][k], pre_calc_length);
                }
            }
        }

        for (int k = 0; k < opt_frame_len; k++)
        {
            kinFactor_connect_hyp_4dof *hypxt = new kinFactor_connect_hyp_4dof(para_local_pose[3][k], para_local_pose[1][k], para_local_pose[2][k], pre_calc_hinge[0], para_agent_centQ[k][0], sigma_hyp_loose);
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<kinFactor_connect_hyp_4dof, 1, 3, 3, 3, 1, 1, 1>(hypxt),
                NULL,
                para_Rt[3][k], para_Rt[1][k], para_Rt[2][k], para_yaw[3][k], para_yaw[1][k], para_yaw[2][k]);
        }
        if (1)
        {
            problem.SetParameterBlockConstant(para_Rt[1][0]);
            problem.SetParameterBlockConstant(para_Rt[2][0]);
            problem.SetParameterBlockConstant(para_Rt[3][0]);
            problem.SetParameterBlockConstant(para_yaw[1][0]);
            problem.SetParameterBlockConstant(para_yaw[2][0]);
            problem.SetParameterBlockConstant(para_yaw[3][0]);
            problem.AddParameterBlock(pre_calc_hinge, 1);
            problem.AddParameterBlock(pre_calc_length, 1);
            problem.SetParameterBlockConstant(pre_calc_hinge);
            problem.SetParameterBlockConstant(pre_calc_length);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_solver_time_in_seconds = 0.2;
        options.max_num_iterations = 12;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        m_buf.unlock();
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