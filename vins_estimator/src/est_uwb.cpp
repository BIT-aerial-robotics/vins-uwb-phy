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

const int USE_TRUE=1;
const int USE_SIM = 1;
const int SOL_LENGTH = 100;
const int MAX_SOL_LENGTH=10000;
const int IMU_PROPAGATE = 1;
const int USE_UWB_INIT = 1;
std::mutex m_buf;
std::map<double, OdometryVins> pose_agent_buf[5];
std::map<double, bool> isPub;
double para_pos[5][MAX_SOL_LENGTH+100][3], para_yaw[5][MAX_SOL_LENGTH+200][1];

double para_HINGE[1] = {0};
double para_LENGTH[1] = {0.841};
double pre_calc_hinge[1] = {para_HINGE[0]};
double pre_calc_length[1] = {para_LENGTH[0]};
double sigma_hyp_loose = 0.01;
double sigma_length_loose = 0.05;
Eigen::Matrix<double, 4, 1> sigma_vins_6dof_loose;
Eigen::Matrix<double, 4, 1> sigma_bet_6dof_loose;
MarginalizationInfo *last_marginalization_info;
vector<double *> last_marginalization_parameter_blocks;
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
    OdometryVins tmp=OdometryVins(ps, vs, ws, qs, time);
    tmp.updateRange(range);
    // if(idx==1){
    //    printf("%lf %lf %lf %lf\n",time,range[0],range[1],range[2]);
    // }
    pose_agent_buf[idx][time] = tmp;
    //pose_agent_buf[idx][time].updateRange(range);
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
Eigen::Vector3d ps[5][MAX_SOL_LENGTH+200], vs[5][MAX_SOL_LENGTH+200], omega[5][MAX_SOL_LENGTH+200];
Eigen::Quaterniond qs[5][MAX_SOL_LENGTH+200];
double alpha[200], para_agent_time[MAX_SOL_LENGTH+200];
double range_mea[5][MAX_SOL_LENGTH+200][10];
double para_anchor[5][3];
double para_anchor_est[5][3];
double para_bias[5][5][1];
ros::Publisher pub_odometry_frame[4];
ros::Publisher pub_odometry_value[4];
Eigen::Vector3d anchor_create_pos[5] = {
    Eigen::Vector3d(-38.17,-34.35,1.38),
    Eigen::Vector3d(32.93,-36.65,3.3),
    Eigen::Vector3d(38.76,46.12,1.59),
    Eigen::Vector3d(-34.48,31.17,1.14)};
std::default_random_engine generator;
std::normal_distribution<double> noise_normal_distribution(0.0, 0.04);

ceres::Problem problem2;
double para_bias_est[5][5][2000][1];
double range_mea_est[5][2000][10];
Eigen::Vector3d tag_pos[5][2000];
int long_window_len;
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
void alignPoints(MatrixXd& pointsA, MatrixXd& pointsB, Matrix3d& rotation, Vector3d& translation) {
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
    if (det < 0) {
        rotation.col(2) *= -1;
    }

    // 计算平移向量
    translation = centroidB - rotation * centroidA;
}

// 计算匹配误差
double computeError(MatrixXd& pointsA, MatrixXd& pointsB, Matrix3d& rotation, Vector3d& translation) {
    int numPoints = pointsA.rows();
    double error = 0.0;

    for (int i = 0; i < numPoints; ++i) {
        Vector3d transformedPoint = rotation * pointsA.row(i).transpose() + translation;
        error += (transformedPoint - pointsB.row(i).transpose()).squaredNorm();
    }

    return error / numPoints;
}
void sync_process()
{
    int last_data_size = 0;
    int sys_cnt = 0;
    int opt_frame_len = 0;
    int faile_num = 0;
    while (1)
    {
        if (faile_num > 1000)
            break;
        TicToc t_sub;
        ros::Rate loop_rate(100);
        loop_rate.sleep();
        m_buf.lock();
        while(pose_agent_buf[1].size() > 0 && opt_frame_len>5){
            if((pose_agent_buf[1].begin()->second.Ps-ps[1][opt_frame_len-1]).norm()<0.1){
                pose_agent_buf[1].erase(pose_agent_buf[1].begin());
            }
            else{
                break;
            }
        }
        while (pose_agent_buf[1].size() > 0 && pose_agent_buf[2].size() > 0 && pose_agent_buf[2].begin()->first - pose_agent_buf[1].begin()->first > 1)
        {
            pose_agent_buf[1].erase(pose_agent_buf[1].begin());
        }
        while (pose_agent_buf[1].size() > 0 && pose_agent_buf[3].size() > 0 && pose_agent_buf[3].begin()->first - pose_agent_buf[1].begin()->first > 1)
        {
            pose_agent_buf[1].erase(pose_agent_buf[1].begin());
        }
        
        int last_opt_frame_len = opt_frame_len;
        //for (int i = 0; i <= 3; i++)
        //    std::cout << pose_agent_buf[i].size() << " ";
        //std::cout << opt_frame_len << " " << faile_num << std::endl;
        while (pose_agent_buf[1].size() > 0)
        {
            // if (faile_num >= 400)
            // {
            //     pose_agent_buf[1].erase(pose_agent_buf[1].begin());
            //     faile_num = 0;
            // }
            double time = pose_agent_buf[1].begin()->first;
            OdometryVins ot[4];
            bool ot_flag[4] = {true, true, false, false};
            ot_flag[2] = OdometryVins::queryOdometryMap(pose_agent_buf[2], time, ot[2], 0.08);
            ot_flag[3] = OdometryVins::queryOdometryMap(pose_agent_buf[3], time, ot[3], 0.08);
            //ot_flag[0] = OdometryVins::queryOdometryMap(pose_agent_buf[0], time, ot[0], 0.08);
            bool flag = ot_flag[0] & ot_flag[1] & ot_flag[2] & ot_flag[3];
            ot[1] = pose_agent_buf[1].begin()->second;
            if (flag == false)
            {
                faile_num++;
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

                    para_pos[2][opt_frame_len][0] = 0;
                    para_pos[2][opt_frame_len][1] = 0;
                    para_pos[2][opt_frame_len][2] = 0;
                    para_yaw[2][opt_frame_len][0] = 0;

                    para_pos[3][opt_frame_len][0] = 0;
                    para_pos[3][opt_frame_len][1] = 0;
                    para_pos[3][opt_frame_len][2] = 0;
                    para_yaw[3][opt_frame_len][0] = 0;
                    // para_pos[1][opt_frame_len][0] = 0;
                    // para_pos[1][opt_frame_len][1] = 0;
                    // para_pos[1][opt_frame_len][2] = 0;
                    // para_yaw[1][opt_frame_len][0] = 0;

                    // para_pos[2][opt_frame_len][0] = -0.478;
                    // para_pos[2][opt_frame_len][1] = -0.828;
                    // para_pos[2][opt_frame_len][2] = 0;
                    // para_yaw[2][opt_frame_len][0] = 120;

                    // para_pos[3][opt_frame_len][0] = 0.478;
                    // para_pos[3][opt_frame_len][1] = -0.828;
                    // para_pos[3][opt_frame_len][2] = 0;
                    // para_yaw[3][opt_frame_len][0] = -120;

                    para_anchor[0][0] = 0, para_anchor[0][1] = 0, para_anchor[0][2] = 0;
                    para_anchor[1][0] = 26.5, para_anchor[1][1] = -0.6, para_anchor[1][2] = 4.5;
                    para_anchor[2][0] = 6.5, para_anchor[2][1] = 10, para_anchor[2][2] = 4.5;
                    para_anchor[3][0] = 11.5, para_anchor[3][1] = 13, para_anchor[3][2] = 4.5;

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
                            // range_mea[i][opt_frame_len][j]/1.8*0.1;
                        }
                    }
                }
                alpha[opt_frame_len] = (ot[0].Rs.toRotationMatrix())(2, 2);
                para_agent_time[opt_frame_len] = time;
                opt_frame_len++;
                while (pose_agent_buf[1].size() > 0 && pose_agent_buf[1].begin()->first - time <= 0.04)
                    pose_agent_buf[1].erase(pose_agent_buf[1].begin());
                // if (opt_frame_len >= SOL_LENGTH)
                //     break;
                if(opt_frame_len-last_opt_frame_len>5)break;
            }
        }
        if (opt_frame_len <= 0||opt_frame_len<=last_opt_frame_len)
        {
            m_buf.unlock();
            faile_num++;
            continue;
        }
        faile_num = 0;
        sys_cnt += 1;
        // std::cout << sys_cnt << " " << opt_frame_len;
        // printf("%lf %lf\n", para_agent_time[0], para_agent_time[opt_frame_len - 1]);
        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        loss_function = new ceres::HuberLoss(1.0);
        int not_memory = 5;
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 0; j < opt_frame_len; j++)
            {
                //problem.AddParameterBlock(para_pos[i][j], 3);
                //problem.AddParameterBlock(para_yaw[i][j], 1);
                //problem.SetParameterBlockConstant(para_pos[i][j]);
                //problem.SetParameterBlockConstant(para_yaw[i][j]);
                for (int k = 0; k <= 3; k++)
                {
                    //if(i==2&&j==opt_frame_len-1)
                    //printf("time=%lf (position %lf %lf %lf %lf %lf %lf)  (range=%lf %lf)\n",para_agent_time[j],ps[i][j].x(),ps[i][j].y(),ps[i][j].z(),
                    //para_anchor[k][0],para_anchor[k][1],para_anchor[k][2],range_mea[i][j][k],(ps[i][j]-anchor_create_pos[k]).norm());
                    UWBFactor_connect_4dof *self_factor = new UWBFactor_connect_4dof(ps[i][j], qs[i][j], pre_calc_hinge[0],range_mea[i][j][k], 0.05);
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<UWBFactor_connect_4dof, 1, 3, 1, 3, 1>(self_factor),
                        loss_function,
                        para_pos[i][0], para_yaw[i][0], para_anchor[k], para_bias[i][k]);
                }
            }
        }
        for (int i = 0; i <= 3; i++)
        {
            for (int j = 0; j < i; j++)
            {
                double dis = (anchor_create_pos[i] - anchor_create_pos[j]).norm()+noise_normal_distribution(generator);
                // printf("dis=== %lf ",dis);
                UWBFactor_anchor_and_anchor *self_factor = new UWBFactor_anchor_and_anchor(dis, 0.05);
                problem2.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<UWBFactor_anchor_and_anchor, 1, 3, 3>(self_factor),
                    NULL, para_anchor[i], para_anchor[j]);
            }
        }
        problem.SetParameterBlockConstant(para_anchor[0]);
        for (int i = 1; i <= 3; i++)
        {
            for (int k = 0; k <= 3; k++)
            {
                problem.AddParameterBlock(para_bias[i][k],1);
                problem.SetParameterBlockConstant(para_bias[i][k]);
                    // UWBBiasFactor *self_factor = new UWBBiasFactor(para_bias[i][k][0], 0.001);
                    // problem.AddResidualBlock(
                    //     new ceres::AutoDiffCostFunction<UWBBiasFactor, 1, 1>(self_factor),
                    //     NULL, para_bias[i][k]);
            }   
        }
        for (int k = 0; k <= 3; k++)
        {
            // problem.AddParameterBlock(para_anchor[k],3);
            // //problem.SetParameterBlockConstant(para_anchor[k]);
            // UWBAnchorFactor *self_factor = new UWBAnchorFactor(para_anchor[k], 1.0 / (log(sys_cnt - 20 + 1) * 0.5 + 1));
            // problem2.AddResidualBlock(
            //     new ceres::AutoDiffCostFunction<UWBAnchorFactor, 3, 3>(self_factor),
            //     NULL, para_anchor[k]);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_solver_time_in_seconds = 0.5;
        options.max_num_iterations = 20;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;

        printf("wdafsufsk  dasflfa");
        if(opt_frame_len>=MAX_SOL_LENGTH){
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
        
        printf("delta T =%lf ",para_agent_time[opt_frame_len-1]-para_agent_time[0]);
        MatrixXd pointsA(4, 3),pointsB(4,3);
        for(int i=0;i<=3;i++){
            pointsA.row(i)<<para_anchor[i][0],para_anchor[i][1],para_anchor[i][2];
            pointsB.row(i)<<anchor_create_pos[i](0),anchor_create_pos[i](1),anchor_create_pos[i](2);
        }
        Matrix3d rotation;
        Vector3d translation;
        alignPoints(pointsA, pointsB, rotation, translation);
        //printf("allign\n");
        // 计算匹配误差
        double error = computeError(pointsA, pointsB, rotation, translation);
        // for(int i=0;i<=3;i++){
        //     printf("xyz (");
        //     for(int k=0;k<=2;k++)
        //     printf("%lf ",para_anchor[i][k]);
        //     Eigen::Vector3d ap=dq*anchor_create_pos[i]+dp;
        //     printf("%lf %lf %lf ",para_anchor[i][0]-ap(0),
        //     para_anchor[i][1]-ap(1),para_anchor[i][2]-ap(2));
        //     printf(")");
        //     // printf(") bias (");
        //     // for(int k=1;k<=3;k++)
        //     // printf("%lf ",para_bias[k][i][0]);
        //     // printf(")");
        //}
        printf("%lf \n",error);
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

    if (USE_SIM)
    {
        para_HINGE[0] = 0.0;
        para_LENGTH[0] = 0.957;
        pre_calc_hinge[0] = para_HINGE[0];
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
    std::thread sync_thread{sync_process};
    ros::spin();
}