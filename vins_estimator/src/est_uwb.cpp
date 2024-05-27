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
#include "factor/marginalization_factor.h"

const int USE_TRUE = 1;
const int USE_SIM = 0;
const int SOL_LENGTH = 100;
const int MAX_SOL_LENGTH = 10000;
const int IMU_PROPAGATE = 1;
const int USE_UWB_INIT = 1;
std::mutex m_buf;
std::map<double, OdometryVins> pose_agent_buf[5];
std::map<double, bool> isPub;
double para_pos[5][MAX_SOL_LENGTH + 100][3], para_yaw[5][MAX_SOL_LENGTH + 200][1];

double para_HINGE[3] = {-0.1, 0, -0.03};
double para_LENGTH[1] = {0.841};
double para_TAG[3] = {-0.1, 0.0, 0.0};
double pre_calc_hinge[3] = {para_HINGE[0], para_HINGE[1], para_HINGE[2]};
double pre_calc_length[1] = {para_LENGTH[0]};
double sigma_hyp_loose = 0.01;
double sigma_length_loose = 0.05;

double opt_eps=0.08;
double opt_des=0.02;
double opt_time_upper=60;
double opt_time_lower=3;
double uwb_bias_beta_low=0.95;
double uwb_bias_beta_upp=1.0;
double uwb_bias_gama_low=0.35;
double uwb_bias_gama_upp=0.44;
double uwb_frame_delta=0.25;
double uwb_outlier=0.2;
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
    double range[10];
    for (int i = 0; i <= 3; i++)
        range[i] = pose_msg->twist.covariance[i];
    OdometryVins tmp = OdometryVins(ps, vs, ws, qs, time);
    tmp.updateRange(range);
    // if(idx==1){
    //    printf("%lf %lf %lf %lf\n",time,range[0],range[1],range[2]);
    // }
    pose_agent_buf[idx][time] = tmp;
    // pose_agent_buf[idx][time].updateRange(range);
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
Eigen::Vector3d ps[5][MAX_SOL_LENGTH + 200], vs[5][MAX_SOL_LENGTH + 200], omega[5][MAX_SOL_LENGTH + 200];
Eigen::Quaterniond qs[5][MAX_SOL_LENGTH + 200];
double alpha[MAX_SOL_LENGTH+200], para_agent_time[MAX_SOL_LENGTH + 200],beta[MAX_SOL_LENGTH+200];
double range_mea[5][MAX_SOL_LENGTH + 200][10];
double para_anchor[5][3];
double para_anchor_est[5][3];
double para_bias[5][5][2];
int tag_data_use[4][MAX_SOL_LENGTH + 200];
double para_anchor_bias[5][5][2];
ros::Publisher pub_odometry_frame[4];
ros::Publisher pub_odometry_value[4];
Eigen::Vector3d anchor_create_pos[5] = {
    Eigen::Vector3d(-4.17, -4.35, 1.38),
    Eigen::Vector3d(2.93, -3.65, 1.3),
    Eigen::Vector3d(2.76, 1.12, 1.59),
    Eigen::Vector3d(-4.48, 1.17, 1.14)};

// Eigen::Vector3d anchor_create_pos[5] = {
//     Eigen::Vector3d(-38.17,-34.35,1.38),
//     Eigen::Vector3d(32.93,-36.65,3.3),
//     Eigen::Vector3d(38.76,46.12,1.59),
//     Eigen::Vector3d(-34.48,31.17,1.14)};
std::default_random_engine generator;
std::normal_distribution<double> noise_normal_distribution(0.05, 0.001);

ceres::Problem problem2;
double para_bias_est[5][5][2000][1];
double range_mea_est[5][2000][10];
Eigen::Vector3d tag_pos[5][2000];
int long_window_len;

ros::Publisher pub_anchor_pos[5];
ros::Publisher pub_ag_rt[4];
ros::Publisher pub_ag_pose[4];
Eigen::Matrix4d distance_anchor;

void readParametersEstUwb(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    if(fsSettings["body_T_hinge"].type()!=cv::FileNode::NONE)
    {
        cv::Mat cv_T;
        fsSettings["body_T_hinge"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        pre_calc_hinge[0]=T(0,3);
        pre_calc_hinge[1]=T(1,3);
        pre_calc_hinge[2]=T(2,3);
    }
    if(fsSettings["distance_anchor"].type()!=cv::FileNode::NONE)
    {
        cv::Mat cv_T;
        fsSettings["distance_anchor"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        distance_anchor=T;
    }
    if(fsSettings["opt_eps"].type()!=cv::FileNode::NONE)
        opt_eps=fsSettings["opt_eps"];
    if(fsSettings["opt_des"].type()!=cv::FileNode::NONE)
        opt_des=fsSettings["opt_des"];
    if(fsSettings["opt_time_upper"].type()!=cv::FileNode::NONE)
        opt_time_upper=fsSettings["opt_time_upper"];
    if(fsSettings["opt_time_lower"].type()!=cv::FileNode::NONE)
        opt_time_lower=fsSettings["opt_time_lower"];
    if(fsSettings["uwb_bias_beta_low"].type()!=cv::FileNode::NONE)
        uwb_bias_beta_low=fsSettings["uwb_bias_beta_low"];
    if(fsSettings["uwb_bias_beta_low"].type()!=cv::FileNode::NONE)
        uwb_bias_beta_low=fsSettings["uwb_bias_beta_low"];
    if(fsSettings["uwb_bias_beta_upp"].type()!=cv::FileNode::NONE)
        uwb_bias_beta_upp=fsSettings["uwb_bias_beta_upp"];
    if(fsSettings["uwb_bias_gama_low"].type()!=cv::FileNode::NONE)
        uwb_bias_gama_low=fsSettings["uwb_bias_gama_low"];
    if(fsSettings["uwb_bias_gama_upp"].type()!=cv::FileNode::NONE)
        uwb_bias_gama_upp=fsSettings["uwb_bias_gama_upp"];
    if(fsSettings["uwb_out"].type()!=cv::FileNode::NONE)
        uwb_bias_gama_upp=fsSettings["uwb_bias_gama_upp"];
}
void pub(int tot, int cnt)
{

    // if (isPub.size() == 0)
    // {
    //     for (int i = 0; i <= 3; i++)
    //     {
    //         for (int j = 0; j < i; j++)
    //         {
    //             double dis = (anchor_create_pos[i] - anchor_create_pos[j]).norm() + noise_normal_distribution(generator);
    //             // printf("dis=== %lf ",dis);
    //             // UWBFactor_anchor_and_anchor *self_factor = new UWBFactor_anchor_and_anchor(dis, 0.2);
    //             // problem2.AddResidualBlock(
    //             //     new ceres::AutoDiffCostFunction<UWBFactor_anchor_and_anchor, 1, 3, 3>(self_factor),
    //             //     NULL, para_anchor_est[i], para_anchor_est[j]);
    //         }
    //     }
    //     // for(int i=1;i<=3;i++){
    //     //     for(int j=0;j<=3;j++){
    //     //         problem2.AddParameterBlock(para_bias_est[i][j][0],1);
    //     //         problem2.SetParameterBlockConstant(para_bias_est[i][j][0])
    //     //     }
    //     // }

    // }
    // for (int i = 0; i < tot; i++)
    // {

    //     if (isPub[para_agent_time[i]] == 0)
    //     {
    //         for (int j = 1; j <= 3; j++)
    //         {
    //             Eigen::Quaterniond lr(Utility::ypr2R(Eigen::Vector3d(para_yaw[j][i][0], 0, 0)));
    //             lr.normalize();
    //             Eigen::Vector3d pos(para_pos[j][i][0], para_pos[j][i][1], para_pos[j][i][2]);
    //             Eigen::Vector3d a_pos = ps[j][i];
    //             Eigen::Quaterniond a_r = qs[j][i];
    //             a_r = lr * a_r;
    //             a_r.normalize();
    //             a_pos = lr * a_pos + pos;

    //             nav_msgs::Odometry odometry;
    //             odometry.header.frame_id = "world";
    //             odometry.header.stamp = ros::Time().fromSec(para_agent_time[i]);
    //             odometry.child_frame_id = "world";
    //             // for(int k=0;k<=3;k++){
    //             //     odometry.twist.covariance[k]=range_mea[j][i][k];
    //             //     printf("%d %d %d range=%lf",j,i,k,range_mea[j][i][k]);
    //             // }
    //             tf::pointEigenToMsg(a_pos, odometry.pose.pose.position);
    //             tf::quaternionEigenToMsg(a_r, odometry.pose.pose.orientation);

    //             pub_odometry_value[j].publish(odometry);

    //             tf::pointEigenToMsg(pos, odometry.pose.pose.position);
    //             tf::quaternionEigenToMsg(lr, odometry.pose.pose.orientation);

    //             pub_odometry_frame[j].publish(odometry);

    //             if (i % 1 == 0 && cnt>=20 &&cnt<=200)
    //             {
    //                 tag_pos[j][long_window_len] = a_pos;
    //                 for (int k = 0; k <= 3; k++)
    //                 {
    //                     range_mea_est[j][long_window_len][k] = range_mea[j][i][k];
    //                     //para_bias_est[j][long_window_len][k][0] = long_window_len == 0 ? range_mea_est[j][long_window_len] / 1.8 * 0.1 : para_bias_est[j][long_window_len - 1][k][0];
    //                     para_bias_est[j][k][long_window_len][0]=0;
    //                     UWBFactor_connect_pos *self_factor = new UWBFactor_connect_pos(
    //                         tag_pos[j][long_window_len], range_mea_est[j][long_window_len][k], 0.05+4*range_mea_est[j][long_window_len][k]*range_mea_est[j][long_window_len][k]);
    //                     problem2.AddResidualBlock(
    //                         new ceres::AutoDiffCostFunction<UWBFactor_connect_pos, 1, 3, 1>(self_factor),
    //                         NULL,
    //                         para_anchor_est[k], para_bias_est[j][k][long_window_len]);
    //                     if (long_window_len >= 1)
    //                     {
    //                         // UWBBiasFactor *bias_factor = new UWBBiasFactor(
    //                         //     para_bias_est[j][k][long_window_len-1][0], 0.005);
    //                         // problem2.AddResidualBlock(
    //                         //     new ceres::AutoDiffCostFunction<UWBBiasFactor, 1, 1>(bias_factor),
    //                         //     NULL,
    //                         //     para_bias_est[j][k][long_window_len]);
    //                         problem2.AddParameterBlock(para_bias_est[j][k][long_window_len],1);
    //                         problem2.SetParameterBlockConstant(para_bias_est[j][k][long_window_len]);
    //                     }
    //                     else{
    //                         problem2.AddParameterBlock(para_bias_est[j][k][long_window_len],1);
    //                         problem2.SetParameterBlockConstant(para_bias_est[j][k][long_window_len]);
    //                     }
    //                 }
    //             }
    //         }
    //         if(i%1==0&& cnt>=20 &&cnt<=200)
    //         long_window_len += 1;
    //     }
    //     isPub[para_agent_time[i]] = 1;
    // }
    // ceres::Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_QR;
    // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // // options.minimizer_progress_to_stdout = true;
    // options.max_solver_time_in_seconds = 3;
    // options.max_num_iterations = 100;
    // ceres::Solver::Summary summary;
    // if (cnt <= 200 && cnt >= 20)
    // {
    //     printf("build finish  long_window_size %d\n",long_window_len);
    //     ceres::Solve(options, &problem2, &summary);
    //     //std::cout << " anchor solve" << summary.FullReport() << std::endl;
    // }

    // for (int i = 0; i <= 3; i++)
    // {
    //     for (int j = 0; j < i; j++)
    //     {
    //         double dis = (Eigen::Vector3d(para_anchor_est[i]) - Eigen::Vector3d(para_anchor_est[j])).norm();
    //         double dis2 = (anchor_create_pos[i] - anchor_create_pos[j]).norm();
    //         printf(" dis = (%d %d %lf %lf)", i, j, dis,dis2);
    //     }
    // }
    // printf("\n");
    // for(int i=0;i<=3;i++){
    //     for (int j = 0; j < i; j++)
    //     {
    //         for(int k=0;k<j;k++){
    //             Eigen::Vector3d d_i_j = (Eigen::Vector3d(para_anchor_est[i]) - Eigen::Vector3d(para_anchor_est[j]));
    //             Eigen::Vector3d d_i_k = (Eigen::Vector3d(para_anchor_est[i]) - Eigen::Vector3d(para_anchor_est[k]));
    //             double alpha=d_i_j.dot(d_i_k)/(d_i_j.norm()*d_i_k.norm());
    //             alpha=acos(alpha)/3.1415*180;

    //             d_i_j = (anchor_create_pos[i] - anchor_create_pos[j]);
    //             d_i_k = (anchor_create_pos[i] - anchor_create_pos[k]);

    //             double beta=d_i_j.dot(d_i_k)/(d_i_j.norm()*d_i_k.norm());
    //             beta=acos(beta)/3.1415*180;
    //             printf(" alpha = (%d %d %lf %lf)", i, j, alpha,beta);
    //         }
    //     }
    // }
    // printf("\n");
    // for(int i=0;i<=3;i++){
    //         printf("xyz (");
    //         for(int k=0;k<=2;k++)
    //         printf("%lf ",para_anchor_est[i][k]);
    //     }
}
void alignPoints(MatrixXd &pointsA, MatrixXd &pointsB, Matrix3d &rotation, Vector3d &translation)
{
    assert(pointsA.rows() == pointsB.rows() && pointsA.cols() == 3 && pointsB.cols() == 3);

    // 计算A和B的中心点
    Vector3d centroidA = pointsA.colwise().mean();
    Vector3d centroidB = pointsB.colwise().mean();

    // 去中心化
    MatrixXd centeredA = pointsA.rowwise() - centroidA.transpose();
    MatrixXd centeredB = pointsB.rowwise() - centroidB.transpose();

    // 计算协方差矩阵
    Matrix3d H = centeredA.transpose() * centeredB;

    // 奇异值分解
    JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
    Matrix3d V = svd.matrixV();
    Matrix3d U = svd.matrixU();

    // 计算旋转矩阵
    rotation = V * U.transpose();

    // 确保旋转矩阵是特殊正交矩阵
    double det = rotation.determinant();
    if (det < 0)
    {
        rotation.col(2) *= -1;
    }

    // 计算平移向量
    translation = centroidB - rotation * centroidA;
}

// 计算匹配误差
double computeError(MatrixXd &pointsA, MatrixXd &pointsB, Matrix3d &rotation, Vector3d &translation,
                    vector<Vector3d> &item)
{
    int numPoints = pointsA.rows();
    double error = 0.0;

    for (int i = 0; i < numPoints; ++i)
    {
        Vector3d transformedPoint = rotation * pointsA.row(i).transpose() + translation;
        item[i] = transformedPoint - pointsB.row(i).transpose();

        error += (transformedPoint - pointsB.row(i).transpose()).norm();
    }

    return error / numPoints;
}
void sync_process()
{
    int last_data_size = 0;
    int sys_cnt = 0;
    int opt_frame_len = 0;
    int faile_num = 0;
    string filename = "/home/f404/est.csv";
    double last_error=10000;
    int init_num=0;
    while (1)
    {
        ofstream file(filename, ios::app);
        // if (faile_num > 1000)
        //     break;
        TicToc t_sub;
        ros::Rate loop_rate(100);
        loop_rate.sleep();
        m_buf.lock();
        // while (pose_agent_buf[1].size() > 0 && opt_frame_len > 5)
        // {
        //     if (pose_agent_buf[1].begin()->first - para_agent_time[opt_frame_len - 1] > 4)
        //     {
        //         break;
        //     }
        //     if ((pose_agent_buf[1].begin()->second.Ps - ps[1][opt_frame_len - 1]).norm() < 0.005)
        //     {
        //         pose_agent_buf[1].erase(pose_agent_buf[1].begin());
        //     }
        //     else
        //     {
        //         break;
        //     }
        // }
        while (pose_agent_buf[1].size() > 0 && pose_agent_buf[2].size() > 0 && pose_agent_buf[2].begin()->first - pose_agent_buf[1].begin()->first > 1)
        {
            pose_agent_buf[1].erase(pose_agent_buf[1].begin());
        }
        while (pose_agent_buf[1].size() > 0 && pose_agent_buf[3].size() > 0 && pose_agent_buf[3].begin()->first - pose_agent_buf[1].begin()->first > 1)
        {
            pose_agent_buf[1].erase(pose_agent_buf[1].begin());
        }

        int last_opt_frame_len = opt_frame_len;
        //printf("receive data begin\n");
        while (pose_agent_buf[1].size() > 0)
        {
            // if (faile_num >= 400)
            // {
            //     pose_agent_buf[1].erase(pose_agent_buf[1].begin());
            //     faile_num = 0;
            // }
            double time = pose_agent_buf[1].begin()->first;
            OdometryVins ot[4];
            bool ot_flag[4] = {false, true, false, false};
            tag_data_use[1][opt_frame_len] = 1;
            ot_flag[2] = OdometryVins::queryOdometryMap(pose_agent_buf[2], time, ot[2], 0.03);
            tag_data_use[2][opt_frame_len] = ot_flag[2];
            ot_flag[3] = OdometryVins::queryOdometryMap(pose_agent_buf[3], time, ot[3], 0.03);
            tag_data_use[3][opt_frame_len] = ot_flag[3];
            ot_flag[0] = OdometryVins::queryOdometryMap(pose_agent_buf[0], time, ot[0], 0.03);
            tag_data_use[0][opt_frame_len] = ot_flag[0];
            bool flag = ot_flag[0] & ot_flag[1] & ot_flag[2] & ot_flag[3];
            ot[1] = pose_agent_buf[1].begin()->second;
            flag = true;
            pose_agent_buf[1].erase(pose_agent_buf[1].begin());
            if (flag == false)
            {
                faile_num++;
                break;
            }
            else
            {
                if (opt_frame_len == 0)
                {
                    double yaw=Utility::R2ypr(ot[0].Rs.toRotationMatrix()).x();
                    yaw=Utility::normalizeAngleByAng(yaw);
                    para_pos[1][opt_frame_len][0] = 0;
                    para_pos[1][opt_frame_len][1] = 0;
                    para_pos[1][opt_frame_len][2] = 0;
                    para_yaw[1][opt_frame_len][0] = 0;

                    // para_pos[2][opt_frame_len][0] = -0.728 ;
                    // para_pos[2][opt_frame_len][1] = 0.420;
                    // para_pos[2][opt_frame_len][2] = 0;
                    // para_yaw[2][opt_frame_len][0] = 120;

                    // para_pos[3][opt_frame_len][0] = -0.728;
                    // para_pos[3][opt_frame_len][1] = -0.420;
                    // para_pos[3][opt_frame_len][2] = 0;
                    // para_yaw[3][opt_frame_len][0] = -120;
                    
                    para_yaw[1][opt_frame_len][0]=yaw;
                    para_pos[2][opt_frame_len][0] = 0.420;//-0.728 ;
                    para_pos[2][opt_frame_len][1] = 0.728;//0.420;
                    para_pos[2][opt_frame_len][2] = 0;
                    para_yaw[2][opt_frame_len][0] = Utility::normalizeAngleByAng(yaw+120);

                    para_pos[3][opt_frame_len][0] = -0.420;//-0.728;
                    para_pos[3][opt_frame_len][1] = 0.728;//-0.420;
                    para_pos[3][opt_frame_len][2] = 0;
                    para_yaw[3][opt_frame_len][0] = Utility::normalizeAngleByAng(yaw-120);

                    para_anchor[0][0] = 0.5, para_anchor[0][1] = 1.0, para_anchor[0][2] = 5;
                    para_anchor[1][0] = 4.5, para_anchor[1][1] = 1.5, para_anchor[1][2] = 2.5;
                    para_anchor[2][0] = 2.5, para_anchor[2][1] = 3, para_anchor[2][2] = 0.8;
                    para_anchor[3][0] = -2.5, para_anchor[3][1] = 0.5, para_anchor[3][2] = 0.7;

                    // para_anchor_est[0][0] = 6.5, para_anchor_est[0][1] = -0.6, para_anchor_est[0][2] = 4.5;
                    // para_anchor_est[1][0] = 20.5, para_anchor_est[1][1] = 10, para_anchor_est[1][2] = 4.5;
                    // para_anchor_est[2][0] = 10.5, para_anchor_est[2][1] = 11, para_anchor_est[2][2] = 4.5;
                    // para_anchor_est[3][0] = 9.5, para_anchor_est[3][1] = 10, para_anchor_est[3][2] = 4.5;
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
                    for (int j = 0; j <= 3; j++)
                    {
                        range_mea[i][opt_frame_len][j] = ot[i].range[j];
                    }
                }

                if (opt_frame_len == 0)
                {
                    for (int i = 1; i <= 3; i++)
                    {
                        for (int j = 0; j <= 3; j++)
                        {
                            para_bias[i][j][0] = 0;
                            para_bias[i][j][1] = 1.00;
                            // range_mea[i][opt_frame_len][j]/1.8*0.1;
                        }
                    }
                }
                alpha[opt_frame_len] = (ot[0].Rs.toRotationMatrix())(2, 2);
                double yaw=Utility::R2ypr(ot[0].Rs.toRotationMatrix()).x();
                beta[opt_frame_len]=(Utility::normalizeAngleByAng(yaw));
                para_agent_time[opt_frame_len] = time;
                opt_frame_len++;
                while (pose_agent_buf[1].size() > 0 && pose_agent_buf[1].begin()->first - time <= 0.02)
                    pose_agent_buf[1].erase(pose_agent_buf[1].begin());
                //  if (opt_frame_len >= SOL_LENGTH)
                //      break;
                if (opt_frame_len - last_opt_frame_len > 2)
                    break;
            }
        }
        
        if (opt_frame_len <= 0 || opt_frame_len <= last_opt_frame_len)
        {
            m_buf.unlock();
            faile_num++;
            continue;
        }
        faile_num = 0;
        sys_cnt += 1;
        // std::cout << sys_cnt << " " << opt_frame_len;
        //printf("receive data finish\n");
        // printf("%lf %lf\n", para_agent_time[0], para_agent_time[opt_frame_len - 1]);
        
        int not_memory = 5;
        
        ceres::Problem problem;
        if (1)
        {
            
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            int upperIdx=opt_frame_len;
            int lowerIdx=max(0,upperIdx-120);
            for (int i = 1; i <= 3; i++)
            {
                for (int k = lowerIdx+1; k < upperIdx; k++)
                {
                    kinFactor_bet_4dof_2 *bxt = new kinFactor_bet_4dof_2(para_pos[i][k], para_yaw[i][k], para_pos[i][k - 1], para_yaw[i][k - 1], para_agent_time[k] - para_agent_time[k - 1], sigma_bet_6dof_loose*(init_num<=50?10:1));
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<kinFactor_bet_4dof_2, 4, 3, 1, 3, 1>(bxt),
                        NULL,
                        para_pos[i][k], para_yaw[i][k], para_pos[i][k - 1], para_yaw[i][k - 1]);
                }
                for (int k = lowerIdx; k < upperIdx; k++)
                {
                    kinFactor_bet_old_4dof_2 *bxt = new kinFactor_bet_old_4dof_2(para_pos[i][k], para_yaw[i][k], sigma_vins_6dof_loose*(init_num<=50?10:1));
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<kinFactor_bet_old_4dof_2, 4, 3, 1>(bxt),
                        NULL,
                        para_pos[i][k], para_yaw[i][k]);
                }
            }
            int len_res_num=0;
            int att_res_num=0;
            for (int i = 1; i <= 3; i++)
            {
                for (int j = 1; j < i; j++)
                {
                    for (int k = lowerIdx; k < upperIdx; k++)
                    {
                        if (tag_data_use[i][k] == 0 || tag_data_use[j][k] == 0)
                            continue;

                        UWBFactor_kin_len *self_factor = new UWBFactor_kin_len(ps[i][k], qs[i][k],
                                                                               ps[j][k], qs[j][k],
                                                                               pre_calc_hinge, para_LENGTH[0], 0.04);
                        problem.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<UWBFactor_kin_len, 1, 3, 1, 3, 1>(self_factor),
                            loss_function,
                            para_pos[i][k], para_yaw[i][k], para_pos[j][k], para_yaw[j][k]);
                        len_res_num+=1;
                    }
                }
                // problem.AddParameterBlock(para_pos[i][0],3);
                // problem.AddParameterBlock(para_yaw[i][0],1);
            }
            for (int k = lowerIdx; k < upperIdx; k++)
            {
                if (tag_data_use[1][k] == 0 || tag_data_use[2][k] == 0 || tag_data_use[3][k] == 0 || tag_data_use[0][k] == 0)
                    continue;

                UWBFactor_kin_att *self_factor = new UWBFactor_kin_att(ps[1][k], qs[1][k],
                                                                       ps[2][k], qs[2][k], ps[3][k], qs[3][k],
                                                                       pre_calc_hinge, alpha[k], 0.01);
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<UWBFactor_kin_att, 1, 3, 1, 3, 1, 3, 1>(self_factor),
                    loss_function,
                    para_pos[1][k], para_yaw[1][k], para_pos[2][k], para_yaw[2][k],
                    para_pos[3][k], para_yaw[3][k]);
                att_res_num+=1;

                {
                   
                    UWBFactor_kin_yaw *yaw_factor = new UWBFactor_kin_yaw(ps[1][k], qs[1][k],
                                                                       ps[2][k], qs[2][k], ps[3][k], qs[3][k],
                                                                       pre_calc_hinge, beta[k], 0.01);
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<UWBFactor_kin_yaw, 1, 3, 1, 3, 1, 3, 1>(yaw_factor),
                        loss_function,
                        para_pos[1][k], para_yaw[1][k], para_pos[2][k], para_yaw[2][k],
                        para_pos[3][k], para_yaw[3][k]);
                    att_res_num+=1;
                }
            }
            problem.SetParameterBlockConstant(para_pos[1][lowerIdx]);
            problem.SetParameterBlockConstant(para_yaw[1][lowerIdx]);
            problem.SetParameterBlockConstant(para_pos[2][lowerIdx]);
            problem.SetParameterBlockConstant(para_yaw[2][lowerIdx]);
            problem.SetParameterBlockConstant(para_pos[3][lowerIdx]);
            problem.SetParameterBlockConstant(para_yaw[3][lowerIdx]);
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.trust_region_strategy_type = ceres::DOGLEG;
            options.max_solver_time_in_seconds = 0.5;
            options.max_num_iterations = 8;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << std::endl;
            init_num+=1;
            //std::cout<<len_res_num<<"  "<<att_res_num<<"  "<<para_agent_time[upperIdx-1]-para_agent_time[lowerIdx]<<std::endl;
        }

        




        if (opt_frame_len>=300&&0)
        {
            ceres::Problem problem2;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            for (int i = 1; i <= 3; i++)
            {

                int use_id=-1;
                for (int j = 0; j < opt_frame_len; j++)
                {
                    if (tag_data_use[i][j] == 0)
                        continue;

                    if(use_id==-1){
                        use_id=j;
                    }
                    else{
                        if((ps[i][j]-ps[i][use_id]).norm()<uwb_frame_delta)continue;
                        else{
                            use_id=j;
                        }
                    }
                    problem2.AddParameterBlock(para_pos[i][j], 3);
                    problem2.AddParameterBlock(para_yaw[i][j], 1);
                    problem2.SetParameterBlockConstant(para_pos[i][j]);
                    problem2.SetParameterBlockConstant(para_yaw[i][j]);
                    for (int k = 0; k <= 3; k++)
                    {
                        // if(i==2&&j==opt_frame_len-1)
                        // printf("time=%lf (position %lf %lf %lf %lf %lf %lf)  (range=%lf %lf)\n",para_agent_time[j],ps[i][j].x(),ps[i][j].y(),ps[i][j].z(),
                        // para_anchor[k][0],para_anchor[k][1],para_anchor[k][2],range_mea[i][j][k],(ps[i][j]-anchor_create_pos[k]).norm());
                        UWBFactor_connect_4dof_plus_mul *self_factor = new UWBFactor_connect_4dof_plus_mul(ps[i][j], qs[i][j],
                                                                                                           para_TAG, range_mea[i][j][k], 0.04);
                        
                        double residual[2];
                        (*self_factor)(para_pos[i][j], para_yaw[i][j], para_anchor[k], para_bias[i][k],residual);
                        if(residual[0]>uwb_outlier/0.04&&opt_frame_len>350)continue;
                        problem2.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<UWBFactor_connect_4dof_plus_mul, 1, 3, 1, 3, 2>(self_factor),
                            loss_function,
                            para_pos[i][j], para_yaw[i][j], para_anchor[k], para_bias[i][k]);
                    }
                    for (int dt = 20; dt <= 80; dt += 25)
                    {

                        if (j - dt < 0)
                            break;
                        for (int k = 0; k <= 3; k++)
                        {
                            int d1 = j, d2 = j - dt;
                            UWBFactor_connect_2time_plus_mul *self_factor = new UWBFactor_connect_2time_plus_mul(ps[i][d1], qs[i][d1],
                                                                                                                 ps[i][d2], qs[i][d2],
                                                                                                                 para_TAG, range_mea[i][d1][k], range_mea[i][d2][k], 0.04);
                            
                            double residual[2];
                            (*self_factor)(para_pos[i][d1], para_yaw[i][d1], 
                                para_pos[i][d2], para_yaw[i][d2],
                                para_anchor[k], para_bias[i][k],residual);
                            if(residual[0]>uwb_outlier/0.04&&opt_frame_len>350)continue;
                            problem2.AddResidualBlock(
                                new ceres::AutoDiffCostFunction<UWBFactor_connect_2time_plus_mul, 1, 3, 1,3,1, 3, 2>(self_factor),
                                loss_function,
                                para_pos[i][d1], para_yaw[i][d1], 
                                para_pos[i][d2], para_yaw[i][d2],
                                para_anchor[k], para_bias[i][k]);
                        }
                    }
                }
            }

            // problem.SetParameterBlockConstant(para_anchor[0]);
            for (int i = 1; i <= 3; i++)
            {
                for (int k = 0; k <= 3; k++)
                {
                    problem2.AddParameterBlock(para_bias[i][k], 2);
                    // problem.SetParameterBlockConstant(para_bias[i][k]);
                    problem2.SetParameterLowerBound(para_bias[i][k], 1, uwb_bias_beta_low);
                    problem2.SetParameterUpperBound(para_bias[i][k], 1, uwb_bias_beta_upp);
                    problem2.SetParameterLowerBound(para_bias[i][k], 0, uwb_bias_gama_low);
                    problem2.SetParameterUpperBound(para_bias[i][k], 0, uwb_bias_gama_upp);
                    // UWBBiasFactor *self_factor = new UWBBiasFactor(para_bias[i][k], 0.01);
                    // problem.AddResidualBlock(
                    //     new ceres::AutoDiffCostFunction<UWBBiasFactor, 2, 2>(self_factor),
                    //     NULL, para_bias[i][k]);
                }
            }
            for (int i = 0; i <= 3; i++)
            {
                for (int j = 0; j < i; j++)
                {
                    double dis = distance_anchor(i,j);
                    if(dis<=1)continue;
                    // printf("dis=== %lf ",dis);
                    UWBFactor_anchor_and_anchor *self_factor = new UWBFactor_anchor_and_anchor(dis, 0.02);
                    problem2.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<UWBFactor_anchor_and_anchor, 1, 3, 3>(self_factor),
                        NULL, para_anchor[i], para_anchor[j]);
                }
            }
            ceres::Solver::Options options2;
            options2.linear_solver_type = ceres::DENSE_SCHUR;
            options2.trust_region_strategy_type = ceres::DOGLEG;
            options2.max_solver_time_in_seconds = 2.5;
            options2.max_num_iterations = 40;
            ceres::Solver::Summary summary2;
            ceres::Solve(options2, &problem2, &summary2);
            std::cout << summary2.BriefReport() << std::endl;
        }
        if (0)
        {

            for (int i = 0; i < ANCHORNUMBER; i++)
            {
                Eigen::MatrixXd A; // Declare matrix A
                Eigen::VectorXd true_x;
                Eigen::VectorXd b; // Declare vector b

                // Assign values to matrix A and vector b (for example)
                int use_uav_num = 1;
                int n = use_uav_num * 6;
                int m = opt_frame_len * use_uav_num;
                b.resize(m);
                A.resize(m, n);
                true_x.resize(m);
                A.setZero();
                b.setZero();

                for (int j = 1; j <= use_uav_num; j++)
                {
                    for (int k = 0; k < opt_frame_len; k++)
                    {
                        Eigen::Matrix3d rot = Utility::fromYawToMat(para_yaw[j][k][0]);
                        Eigen::Vector3d tran(para_pos[j][k]);
                        tran = rot * ps[j][k] + tran;
                        b.coeffRef((j - 1) * (opt_frame_len) + k) = range_mea[j][k][i] * range_mea[j][k][i];

                        A.coeffRef((j - 1) * (opt_frame_len) + k, (j - 1) * 6 + 0) = -2 * tran.x();
                        A.coeffRef((j - 1) * (opt_frame_len) + k, (j - 1) * 6 + 1) = -2 * tran.y();
                        A.coeffRef((j - 1) * (opt_frame_len) + k, (j - 1) * 6 + 2) = -2 * tran.z();
                        A.coeffRef((j - 1) * (opt_frame_len) + k, (j - 1) * 6 + 3) = tran.norm() * tran.norm();
                        A.coeffRef((j - 1) * (opt_frame_len) + k, (j - 1) * 6 + 4) = 2 * range_mea[j][k][i];
                        A.coeffRef((j - 1) * (opt_frame_len) + k, (j - 1) * 6 + 5) = 1;
                    }
                    // true_x.coeffRef((j-1)*6+0)=1.05*1.05*anchor_create_pos[i].x();
                    // true_x.coeffRef((j-1)*6+1)=1.05*1.05*anchor_create_pos[i].y();
                    // true_x.coeffRef((j-1)*6+2)=1.05*1.05*anchor_create_pos[i].z();
                    // true_x.coeffRef((j-1)*6+3)=1.05*1.05;
                    // true_x.coeffRef((j-1)*6+4)=0;
                    // true_x.coeffRef((j-1)*6+5)=1.05*1.05*(anchor_create_pos[i].norm()*anchor_create_pos[i].norm());
                }
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
                // Solve linear equations Ax = b
                Eigen::VectorXd x = svd.solve(b);
                // Eigen::VectorXd ans=A*true_x-b;
                // cout<<ans<<endl;
                for (int j = 1; j <= 3; j++)
                {
                    double beta2 = x.coeffRef((j - 1) * 6 + 3);
                    double beta = sqrt(beta2);
                    double gama = x.coeffRef((j - 1) * 6 + 4);
                    double qx = x.coeffRef((j - 1) * 6 + 0) / beta2;
                    double qy = x.coeffRef((j - 1) * 6 + 1) / beta2;
                    double qz = x.coeffRef((j - 1) * 6 + 2) / beta2;
                    para_anchor[i][0]+=qx;
                    para_anchor[i][1]+=qy;
                    para_anchor[i][2]+=qz;
                    para_bias[j][i][0]=gama;
                    para_bias[j][i][1]=beta;
                }
                para_anchor[i][0]/=3;
                para_anchor[i][1]/=3;
                para_anchor[i][2]/=3;
                printf("init val %lf %lf %lf",para_anchor[i][0],para_anchor[i][1],para_anchor[i][2]);
                for(int j=1;j<=use_uav_num;j++)
                printf("(bias %lf %lf)",para_bias[j][i][0],para_bias[j][i][1]);
                printf("\n");
            }
        }
        // printf("wdafsufsk  dasflfa");
        if (opt_frame_len >= MAX_SOL_LENGTH)
        {
            // for (int i = 0; i < opt_frame_len - not_memory; i++)
            // {
            //     for (int j = 1; j <= 3; j++)
            //     {
            //         para_yaw[j][i][0] = para_yaw[j][i + not_memory][0];
            //         for (int k = 0; k <= 2; k++)
            //             para_pos[j][i][k] = para_pos[j][i + not_memory][k];
            //         ps[j][i] = ps[j][i + not_memory];
            //         vs[j][i] = vs[j][i + not_memory];
            //         omega[j][i] = omega[j][i + not_memory];
            //         qs[j][i] = qs[j][i + not_memory];
            //         alpha[i] = alpha[i + not_memory];
            //         para_agent_time[i] = para_agent_time[i + not_memory];
            //         for (int k = 0; k <= 3; k++)
            //         {
            //             range_mea[j][i][k] = range_mea[j][i + not_memory][k];
            //         }
            //     }
            // }
            // opt_frame_len -= not_memory;
            break;
        }
        m_buf.unlock();

        printf("delta T =%lf ", para_agent_time[opt_frame_len - 1] - para_agent_time[0]);
        MatrixXd pointsA(4, 3), pointsB(4, 3);
        for (int i = 0; i <= 3; i++)
        {
            pointsA.row(i) << para_anchor[i][0], para_anchor[i][1], para_anchor[i][2];
            pointsB.row(i) << anchor_create_pos[i](0), anchor_create_pos[i](1), anchor_create_pos[i](2);
        }
        Matrix3d rotation;
        Vector3d translation;
        alignPoints(pointsA, pointsB, rotation, translation);
        // printf("allign\n");
        //  计算匹配误差
        vector<Eigen::Vector3d> item_error(4, Eigen::Vector3d::Zero());
        double error = computeError(pointsA, pointsB, rotation, translation, item_error);
        string con = "";
        double delta_time=para_agent_time[opt_frame_len - 1] - para_agent_time[0];
        //con = to_string(delta_time) + ",";
        // for (int i = 0; i < 4; i++)
        // {
        //     con = con + to_string(item_error[i].norm()) + ",";
        //     for (int j = 0; j <= 2; j++)
        //         con = con + to_string(item_error[i](j)) + ",";
        // }
        //con = con + to_string(error) + "\n";
        //file << con;
        printf("%d ", opt_frame_len);
        // for (int i = 0; i <= 3; i++)
        // {
        //     printf("xyz (");
        //     for (int k = 0; k <= 2; k++)
        //         printf("%lf ", para_anchor[i][k]);
        //     // Eigen::Vector3d ap=dq*anchor_create_pos[i]+dp;
        //     // printf("%lf %lf %lf ",para_anchor[i][0]-ap(0),
        //     // para_anchor[i][1]-ap(1),para_anchor[i][2]-ap(2));
        //     // printf(")");
        //     // printf(") bias (");
        //     // for(int k=1;k<=3;k++)
        //     // printf("%lf ",para_bias[k][i][0]);
        //     // printf(")");
        // }
        //printf("error1==%lf \n", error);
        // for (int i = 1; i <= 3; i++)
        // {
        //     for (int j = 0; j <= 3; j++)
        //     {
        //         printf("%lf %lf ", para_bias[i][j][1], para_bias[i][j][0]);
        //     }
        // }
        // printf("\n");
        // for (int i = 1; i <= 3; i++)
        // {
        //     printf("%lf %lf %lf %lf", para_yaw[i][0]);
        // }
        double anchor_error=error;
        //file.close();
        error = 0;
        for (int i = 1; i <= 3; i++)
        {
            double err_agent = 0, err_agent_tag[5] = {0, 0, 0, 0, 0};
            int cnt_tag[5]={1,1,1,1,1};
            for (int j = 0; j < opt_frame_len; j++)
            {
                if (tag_data_use[i][j] == 0)
                    continue;
                for (int k = 0; k <= 3; k++)
                {
                    UWBFactor_connect_4dof_plus_mul *self_factor = new UWBFactor_connect_4dof_plus_mul(ps[i][j], qs[i][j],
                                                                                                       para_TAG, range_mea[i][j][k], 1);
                    double res[2];
                    (*self_factor)(para_pos[i][j], para_yaw[i][j], para_anchor[k], para_bias[i][k], res);
                    if(res[0]>uwb_outlier)continue;
                    err_agent_tag[k] += abs(res[0]);
                    cnt_tag[k]++;
                }
            }
            for (int k = 0; k <= 3; k++)
            {
                err_agent_tag[k] /= cnt_tag[k];
                // printf("tag%d,err_mean=%lf ",k,err_agent_tag[k]);
                err_agent += err_agent_tag[k];
            }
            err_agent /= 4;
            printf("alltag %d err_mean===%lf   ",i,err_agent);
            error += err_agent;
        }
        error/=3;
        printf("allagent err_all %lf\n", error);
        for (int j = 1; j <= 3; j++)
        {
            ROS_INFO("(%lf %lf %lf %lf)", para_pos[j][opt_frame_len - 1][0], para_pos[j][opt_frame_len - 1][1],
                     para_pos[j][opt_frame_len - 1][2], para_yaw[j][opt_frame_len - 1][0]);
        }
        auto check=[last_error,error](){
            if(last_error-error<=0.001&&last_error-error>0)return true;
            if((last_error-error)/last_error<=opt_des)return true;
            if(error<=opt_eps)return true;
            return false;
        };
        if (check()&&delta_time>opt_time_lower&&0)
        {
            ROS_INFO("begin cout matrix and anchor");
            for (int i = 0; i < ANCHORNUMBER; i++)
            {
                nav_msgs::Odometry dat;
                dat.header.stamp=ros::Time(para_agent_time[opt_frame_len-1]);
                dat.pose.pose.position.x = para_anchor[i][0];
                dat.pose.pose.position.y = para_anchor[i][1];
                dat.pose.pose.position.z = para_anchor[i][2];
                for (int j = 1; j <= 3; j++)
                {
                    dat.twist.covariance[(j - 1) * 2 + 0] = para_bias[j][i][0],
                                                       dat.twist.covariance[(j - 1) * 2 + 1] = para_bias[j][i][1];
                    cout<<i<<" "<<j<<" "<<para_bias[j][i][0]<<"  "<<para_bias[j][i][1]<<endl;
                }
                pub_anchor_pos[i].publish(dat);
            }
            for(int i=1;i<=3;i++){
                geometry_msgs::PoseStamped rt_world;
                rt_world.header.stamp=ros::Time(para_agent_time[opt_frame_len-1]);
                Eigen::Vector3d pos(para_pos[i][opt_frame_len-1]);
                Eigen::Quaterniond q{Utility::fromYawToMat(para_yaw[i][opt_frame_len-1][0])};
                tf::pointEigenToMsg(pos,rt_world.pose.position);
                tf::quaternionEigenToMsg(q,rt_world.pose.orientation);
                pub_ag_rt[i].publish(rt_world);
            }
            break;
        }
        for(int i=1;i<=3;i++){
            nav_msgs::Odometry rt_world;
            rt_world.header.stamp=ros::Time(para_agent_time[opt_frame_len-1]);
            Eigen::Vector3d pos(para_pos[i][opt_frame_len-1]);
            Eigen::Quaterniond q{Utility::fromYawToMat(para_yaw[i][opt_frame_len-1][0])};
            pos=q*ps[i][opt_frame_len-1]+pos;
            q=q*qs[i][opt_frame_len-1];
            q.normalize();
            Eigen::Vector3d v=q*vs[i][opt_frame_len-1];
            tf::pointEigenToMsg(pos,rt_world.pose.pose.position);
            tf::quaternionEigenToMsg(q,rt_world.pose.pose.orientation);
            tf::vectorEigenToMsg(v,rt_world.twist.twist.linear);
            pub_ag_pose[i].publish(rt_world);
        }
        if(delta_time>opt_time_upper){
            //break;
        }
        last_error=error;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "loose");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // if(argc==2){
    //     string config_file = argv[1];
    //     printf("config_file: %s\n", argv[1]);
    //     //readParametersEstUwb(config_file);
    // }
    
    ros::Subscriber sub_agent1_pose, sub_agent2_pose, sub_agent3_pose;
    ros::Subscriber sub_agent0_imu;

    sigma_bet_6dof_loose(0) = sigma_bet_6dof_loose(1) = sigma_bet_6dof_loose(2) = 0.005;
    sigma_bet_6dof_loose(3) = 0.1;
    sigma_vins_6dof_loose(0) = sigma_vins_6dof_loose(1) = sigma_vins_6dof_loose(2) = 0.1;
    sigma_vins_6dof_loose(3) = 1;
    if (USE_SIM == 0)
        sub_agent0_imu = n.subscribe("/mavros/imu/data", 2000, center_pose_callback);
    else
        sub_agent0_imu = n.subscribe("/imu_0", 2000, center_pose_callback);

    if (USE_SIM)
    {
        para_HINGE[2] = 0.0;
        para_LENGTH[0] = 0.957;
        pre_calc_hinge[2] = para_HINGE[2];
        pre_calc_length[0] = para_LENGTH[0];
    }
    if (IMU_PROPAGATE == 1)
    {
        sub_agent1_pose = n.subscribe<nav_msgs::Odometry>("/ag1/vins_estimator/imu_propagate_noworld", 2000, boost::bind(agent_pose_callback, _1, 1));
        sub_agent2_pose = n.subscribe<nav_msgs::Odometry>("/ag2/vins_estimator/imu_propagate_noworld", 2000, boost::bind(agent_pose_callback, _1, 2));
        sub_agent3_pose = n.subscribe<nav_msgs::Odometry>("/ag3/vins_estimator/imu_propagate_noworld", 2000, boost::bind(agent_pose_callback, _1, 3));
    }
    else
    {
        sub_agent1_pose = n.subscribe<nav_msgs::Odometry>("/ag1/odometry", 2000, boost::bind(agent_pose_callback, _1, 1));
        sub_agent2_pose = n.subscribe<nav_msgs::Odometry>("/ag2/odometry", 2000, boost::bind(agent_pose_callback, _1, 2));
        sub_agent3_pose = n.subscribe<nav_msgs::Odometry>("/ag3/odometry", 2000, boost::bind(agent_pose_callback, _1, 3));
    }
    for (int i = 0; i < ANCHORNUMBER; i++)
    {
        pub_anchor_pos[i] = n.advertise<nav_msgs::Odometry>("/anchor_pos" + std::to_string(i), 500);
    }

    for (int i = 1; i <= 3; i++)
    {
        pub_ag_rt[i] = n.advertise<geometry_msgs::PoseStamped>("/ag" + std::to_string(i) + "/rt_world", 500);
        pub_ag_pose[i] = n.advertise<nav_msgs::Odometry>("/ag" + std::to_string(i) + "/calib_pose", 500);
    }
    std::thread sync_thread{sync_process};
    ros::spin();
}