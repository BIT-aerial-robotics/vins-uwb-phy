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

const int USE_SIM = 1;
const int SOL_LENGTH = 100;
const int IMU_PROPAGATE = 1;
std::mutex m_buf;
std::map<double, OdometryVins> pose_agent_buf[5];
std::map<double, bool> isPub;
double para_pos[5][200][3], para_yaw[5][200][1];

double para_HINGE[1] = {-0.09};
double para_LENGTH[1] = {0.841};
double pre_calc_hinge[1] = {para_HINGE[0]};
double pre_calc_length[1] = {para_LENGTH[0]};
double sigma_hyp_loose = 0.02;
double sigma_length_loose = 0.04;
Eigen::Matrix<double, 4, 1> sigma_vins_6dof_loose;
Eigen::Matrix<double, 4, 1> sigma_bet_6dof_loose;
// int rev_cnt[5];
void agent_pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg, const int &idx)
{
    m_buf.lock();
    Eigen::Vector3d ws;
    Eigen::Quaterniond qs;
    Eigen::Vector3d ps = Eigen::Vector3d::Zero(), vs = Eigen::Vector3d::Zero();
    tf::quaternionMsgToEigen(pose_msg->pose.pose.orientation, qs);
    tf::vectorMsgToEigen(pose_msg->twist.twist.angular, ws);
    tf::vectorMsgToEigen(pose_msg->twist.twist.linear, vs);
    tf::pointMsgToEigen(pose_msg->pose.pose.position, ps);
    double time = pose_msg->header.stamp.toSec();
    pose_agent_buf[idx][time] = OdometryVins(ps, vs, ws, qs, time);
    m_buf.unlock();
}
void center_pose_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    m_buf.lock();
    Eigen::Vector3d ws;
    Eigen::Quaterniond qs;
    Eigen::Vector3d ps = Eigen::Vector3d::Zero(), vs = Eigen::Vector3d::Zero();
    tf::vectorMsgToEigen(imu_msg->angular_velocity, ws);
    tf::quaternionMsgToEigen(imu_msg->orientation, qs);
    double time = imu_msg->header.stamp.toSec();
    // std::cout<<time<<std::endl;
    pose_agent_buf[0][time] = OdometryVins(ps, vs, ws, qs, time);
    m_buf.unlock();
}
Eigen::Vector3d ps[5][200], vs[5][200], omega[5][200];
Eigen::Quaterniond qs[5][200];
double alpha[200], para_agent_time[200];
ros::Publisher pub_odometry_frame[4];
ros::Publisher pub_odometry_value[4];
void pub(int tot)
{
    for (int i = 0; i < tot; i++)
    {

        if (isPub[para_agent_time[i]] == 0)
        {
            for (int j = 1; j <= 3; j++)
            {
                Eigen::Quaterniond lr(Utility::ypr2R(Eigen::Vector3d(para_yaw[j][i][0], 0, 0)));
                lr.normalize();
                Eigen::Vector3d pos(para_pos[j][i][0], para_pos[j][i][1], para_pos[j][i][2]);
                Eigen::Vector3d a_pos = ps[j][i];
                Eigen::Quaterniond a_r = qs[j][i];
                a_r = lr * a_r;
                a_r.normalize();
                a_pos = lr * a_pos + pos;

                nav_msgs::Odometry odometry;
                odometry.header.frame_id = "world";
                odometry.header.stamp = ros::Time().fromSec(para_agent_time[i]);
                odometry.child_frame_id = "world";
                tf::pointEigenToMsg(a_pos, odometry.pose.pose.position);
                tf::quaternionEigenToMsg(a_r, odometry.pose.pose.orientation);

                pub_odometry_value[j].publish(odometry);

                tf::pointEigenToMsg(pos, odometry.pose.pose.position);
                tf::quaternionEigenToMsg(lr, odometry.pose.pose.orientation);

                pub_odometry_frame[j].publish(odometry);
            }
        }
        isPub[para_agent_time[i]]=1;
    }
}
void sync_process()
{
    int last_data_size = 0;
    int sys_cnt = 0;
    int opt_frame_len = 0;
    int faile_num = 0;

    while (1)
    {
        if(faile_num>1000)break;
        TicToc t_sub;
        ros::Rate loop_rate(100);
        loop_rate.sleep();
        m_buf.lock();
        while (pose_agent_buf[1].size() > 0 && pose_agent_buf[2].size() > 0 && pose_agent_buf[2].begin()->first - pose_agent_buf[1].begin()->first > 0.08)
        {
            pose_agent_buf[1].erase(pose_agent_buf[1].begin());
        }
        while (pose_agent_buf[1].size() > 0 && pose_agent_buf[3].size() > 0 && pose_agent_buf[3].begin()->first - pose_agent_buf[1].begin()->first > 0.08)
        {
            pose_agent_buf[1].erase(pose_agent_buf[1].begin());
        }
        int last_opt_frame_len = opt_frame_len;
        for (int i = 0; i <= 3; i++)std::cout << pose_agent_buf[i].size() << " ";
        std::cout << opt_frame_len << " " << faile_num << std::endl;
        while (pose_agent_buf[1].size() > 0 && opt_frame_len < SOL_LENGTH)
        {
            // if (faile_num >= 400)
            // {
            //     pose_agent_buf[1].erase(pose_agent_buf[1].begin());
            //     faile_num = 0;
            // }
            double time = pose_agent_buf[1].begin()->first;
            OdometryVins ot[4];
            bool ot_flag[4] = {false, true, false, false};
            ot_flag[2] = OdometryVins::queryOdometryMap(pose_agent_buf[2], time, ot[2], 0.08);
            ot_flag[3] = OdometryVins::queryOdometryMap(pose_agent_buf[3], time, ot[3], 0.08);
            ot_flag[0] = OdometryVins::queryOdometryMap(pose_agent_buf[0], time, ot[0], 0.08);
            bool flag = ot_flag[0] & ot_flag[1] & ot_flag[2] & ot_flag[3];
            ot[1] = pose_agent_buf[1].begin()->second;
            if (flag == false)
            {
                //faile_num++;
                break;
            }
            else
            {
                if (opt_frame_len == 0)
                {
                    para_pos[1][opt_frame_len][0] = 0;
                    para_pos[1][opt_frame_len][1] = 0;
                    para_pos[1][opt_frame_len][2] = 0;
                    para_yaw[1][opt_frame_len][0] = 0;
                    para_pos[2][opt_frame_len][0] = -0.4;
                    para_pos[2][opt_frame_len][1] = -0.7;
                    para_pos[2][opt_frame_len][2] = 0;
                    para_yaw[2][opt_frame_len][0] = 120;
                    para_pos[3][opt_frame_len][0] = 0.4;
                    para_pos[3][opt_frame_len][1] = -0.7;
                    para_pos[3][opt_frame_len][2] = 0;
                    para_yaw[3][opt_frame_len][0] = -120;
                }
                else
                {
                    for (int i = 1; i <= 3; i++)
                    {
                        for (int j = 0; j <= 2; j++)
                            para_pos[i][opt_frame_len][j] = para_pos[i][opt_frame_len - 1][j];
                        para_yaw[i][opt_frame_len][0] = para_yaw[i][opt_frame_len - 1][0];
                    }
                }
                for (int i = 1; i <= 3; i++)
                {
                    ps[i][opt_frame_len] = ot[i].Ps;
                    vs[i][opt_frame_len] = ot[i].Vs;
                    omega[i][opt_frame_len] = ot[i].Ws;
                    qs[i][opt_frame_len] = ot[i].Rs;
                }
                alpha[opt_frame_len] = (ot[0].Rs.toRotationMatrix())(2, 2);
                para_agent_time[opt_frame_len] = time;
                opt_frame_len++;
                while (pose_agent_buf[1].size() > 0 && pose_agent_buf[1].begin()->first - time <= 0.02)
                    pose_agent_buf[1].erase(pose_agent_buf[1].begin());
                if (opt_frame_len >= SOL_LENGTH)
                    break;
            }
        }
        if (opt_frame_len < SOL_LENGTH)
        {
            m_buf.unlock();
            faile_num++;
            continue;
        }
        faile_num=0;
        sys_cnt += 1;
        std::cout << sys_cnt << " " << opt_frame_len << std::endl;
        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        loss_function = new ceres::HuberLoss(1.0);
        int not_memory = 5;
        for (int i = 1; i <= 3; i++)
        {

            for (int k = not_memory; k < opt_frame_len; k++)
            {
                kinFactor_bet_4dof_2 *bxt = new kinFactor_bet_4dof_2(para_pos[i][k], para_yaw[i][k], para_pos[i][k - not_memory], para_yaw[i][k - not_memory], para_agent_time[k] - para_agent_time[k-not_memory], sigma_bet_6dof_loose);
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<kinFactor_bet_4dof_2, 4, 3, 1, 3, 1>(bxt),
                    NULL,
                    para_pos[i][k], para_yaw[i][k], para_pos[i][k - not_memory], para_yaw[i][k - not_memory]);
            }
        }
        for (int i = 1; i <= 3; i++)
        {

            for (int k = 0; k < opt_frame_len; k++)
            {
                kinFactor_bet_old_4dof_2 *bxt = new kinFactor_bet_old_4dof_2(para_pos[i][k], para_yaw[i][k], sigma_vins_6dof_loose);
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<kinFactor_bet_old_4dof_2, 4, 3, 1>(bxt),
                    NULL,
                    para_pos[i][k], para_yaw[i][k]);
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
                    kinFactor_connect_4dof_2 *self_factor = new kinFactor_connect_4dof_2(ps[i][k], qs[i][k], ps[j][k], qs[j][k], pre_calc_hinge[0], sigma_length_loose);
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<kinFactor_connect_4dof_2, 1, 3, 3, 1, 1, 1>(self_factor),
                        NULL,
                        para_pos[i][k], para_pos[j][k], para_yaw[i][k], para_yaw[j][k], pre_calc_length);
                }
            }
        }

        for (int k = 0; k < opt_frame_len; k++)
        {
            kinFactor_connect_hyp_4dof_2 *hypxt = new kinFactor_connect_hyp_4dof_2(ps[1][k], qs[1][k], ps[2][k], qs[2][k], ps[3][k], qs[3][k],
                                                                                   pre_calc_hinge[0], alpha[k], sigma_hyp_loose);
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<kinFactor_connect_hyp_4dof_2, 1, 3, 3, 3, 1, 1, 1>(hypxt),
                NULL,
                para_pos[1][k], para_pos[2][k], para_pos[3][k], para_yaw[1][k], para_yaw[2][k], para_yaw[3][k]);
        }
        if (1)
        {
            problem.SetParameterBlockConstant(para_pos[1][0]);
            problem.SetParameterBlockConstant(para_pos[2][0]);
            problem.SetParameterBlockConstant(para_pos[3][0]);
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
        options.max_solver_time_in_seconds = 0.1;
        options.max_num_iterations = 8;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
        pub(opt_frame_len);
        
        for (int i = 0; i < opt_frame_len - not_memory; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                para_yaw[j][i][0] = para_yaw[j][i + not_memory][0];
                for (int k = 0; k <= 2; k++)
                    para_pos[j][i][k] = para_pos[j][i + not_memory][k];
                ps[j][i] = ps[j][i + not_memory];
                vs[j][i] = vs[j][i + not_memory];
                omega[j][i] = omega[j][i + not_memory];
                qs[j][i] = qs[j][i + not_memory];
                alpha[i] = alpha[i + not_memory];
                para_agent_time[i] = para_agent_time[i + not_memory];
            }
        }
        opt_frame_len -= not_memory;
        m_buf.unlock();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "loose");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::Subscriber sub_agent1_pose, sub_agent2_pose, sub_agent3_pose;
    ros::Subscriber sub_agent0_imu;
    if (USE_SIM == 0)
        sub_agent0_imu = n.subscribe("/mavros/imu/data", 2000, center_pose_callback);
    else
        sub_agent0_imu = n.subscribe("/imu_0", 2000, center_pose_callback);
    
    if(USE_SIM){
        para_HINGE[0]=0.0;
        para_LENGTH[0]=0.957;
        pre_calc_hinge[0]=para_HINGE[0];
        pre_calc_length[0]=para_LENGTH[0];

    }
    if (IMU_PROPAGATE == 1)
    {
        sub_agent1_pose = n.subscribe<nav_msgs::Odometry>("/ag1/imu_propagate", 2000, boost::bind(agent_pose_callback, _1, 1));
        sub_agent2_pose = n.subscribe<nav_msgs::Odometry>("/ag2/imu_propagate", 2000, boost::bind(agent_pose_callback, _1, 2));
        sub_agent3_pose = n.subscribe<nav_msgs::Odometry>("/ag3/imu_propagate", 2000, boost::bind(agent_pose_callback, _1, 3));
    }
    else
    {
        sub_agent1_pose = n.subscribe<nav_msgs::Odometry>("/ag1/odometry", 2000, boost::bind(agent_pose_callback, _1, 1));
        sub_agent2_pose = n.subscribe<nav_msgs::Odometry>("/ag2/odometry", 2000, boost::bind(agent_pose_callback, _1, 2));
        sub_agent3_pose = n.subscribe<nav_msgs::Odometry>("/ag3/odometry", 2000, boost::bind(agent_pose_callback, _1, 3));
    }
    sigma_bet_6dof_loose(0) = sigma_bet_6dof_loose(1) = sigma_bet_6dof_loose(2) = 0.005;
    sigma_bet_6dof_loose(3) = 0.08;
    sigma_vins_6dof_loose(0) = sigma_vins_6dof_loose(1) = sigma_vins_6dof_loose(2) = 0.05;
    sigma_vins_6dof_loose(3) = 0.5;
    pub_odometry_frame[1] = n.advertise<nav_msgs::Odometry>("/ag1/rt_2_world", 1000);
    pub_odometry_frame[2] = n.advertise<nav_msgs::Odometry>("/ag2/rt_2_world", 1000);
    pub_odometry_frame[3] = n.advertise<nav_msgs::Odometry>("/ag3/rt_2_world", 1000);

    pub_odometry_value[1] = n.advertise<nav_msgs::Odometry>("/ag1/odometry_calib", 1000);
    pub_odometry_value[2] = n.advertise<nav_msgs::Odometry>("/ag2/odometry_calib", 1000);
    pub_odometry_value[3] = n.advertise<nav_msgs::Odometry>("/ag3/odometry_calib", 1000);
    std::thread sync_thread{sync_process};
    ros::spin();
}