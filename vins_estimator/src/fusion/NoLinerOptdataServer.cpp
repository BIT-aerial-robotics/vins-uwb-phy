
#include <ros/ros.h>
#include "vins/subdata.h"
#include "vins/reqdata.h"
#include "vins/timedata.h"
#include "vins/accdata.h"
#include "vins/wrpdata.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include<geometry_msgs/Quaternion.h>
#include <iterator>

#include <mutex>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include "../factor/pose_local_parameterization.h"
#include "../estimator/parameters.h"
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include "../utility/visualization.h"
#include "../utility/utility.h"
#include <random>
#include <iomanip>



int sim_flag=1;
int big_uwb_noise=0;
int lose_uwb_signal=0;
int big_uwb_noise_number[5];
std::mutex m_buf;
ros::Publisher ang_calib;
TicToc ag[5];
double ag_Time[5]={192608170000,192608170000,192608170000,192608170000,192608170000};
struct dataType
{
    double header;
    double pix;
    geometry_msgs::Vector3 Ps,Vs,Bas,Bgs,AgPs,AgVs;
    geometry_msgs::Quaternion Rs;
    tf::Vector3 P,V,Ba,Bg,AP,AV;
    tf::Quaternion R;
};
std::vector< std::map<double,dataType> > data(5);
std::map<double , Vector3d > cent_ang_map;
std::map<double,Vector3d>cent_acc_map;
std::map<double,Eigen::Quaterniond >cent_gt_map;
std::map<double,Eigen::Quaterniond >cent_imu_map;
std::queue <pair<std_msgs::Header,Eigen::Quaterniond> > q_cent_pose;
Eigen:: Matrix3d R_imu_to_gt;
Eigen::Quaterniond Q_imu_to_gt;
std::default_random_engine e{1};
std::normal_distribution<double> u{0,0.05};
std::default_random_engine ce{2};
std::normal_distribution<double> cu{0,0.3};
std::default_random_engine qe{3};
std::normal_distribution<double> qu{0,0.015};
//uniform_real_distribution<double> u(-0.03,0.03);
int flagCalcTime[5][5];
bool flag_imu_to_gt=false;
struct UWBType{
    geometry_msgs::PoseStamped ps;
    double dis[5];
};
Eigen::Vector3d AnchorPos[5]{
    Eigen::Vector3d(10,0,-1.5),
    Eigen::Vector3d(5,-5,5.4),
    Eigen::Vector3d(-6,-9,8.7),
    Eigen::Vector3d(0,10,7.5),
    Eigen::Vector3d(5,4,3.0)
};
nav_msgs::Path no_path_o[4];
ros::Publisher no_pub_path[4];


ros::Publisher uwb_mean_data[4];

std::map<double,UWBType>gt[5];
std::map<double,UWBType>UWBMean[5];
inline void tf2msg(dataType& tmp)
{
    tmp.Ps.x=tmp.P.getX();
    tmp.Ps.y=tmp.P.getY();
    tmp.Ps.z=tmp.P.getZ();

    tmp.AgPs.x=tmp.AP.getX();
    tmp.AgPs.y=tmp.AP.getY();
    tmp.AgPs.z=tmp.AP.getZ();

    tmp.Vs.x=tmp.V.getX();
    tmp.Vs.y=tmp.V.getY();
    tmp.Vs.z=tmp.V.getZ();

    tmp.AgVs.x=tmp.AV.getX();
    tmp.AgVs.y=tmp.AV.getY();
    tmp.AgVs.z=tmp.AV.getZ();


    tmp.Bas.x=tmp.Ba.getX();
    tmp.Bas.y=tmp.Ba.getY();
    tmp.Bas.z=tmp.Ba.getZ();
    tmp.Bgs.x=tmp.Bg.getX();
    tmp.Bgs.y=tmp.Bg.getY();
    tmp.Bgs.z=tmp.Bg.getZ();
    tmp.R.normalize();
    tf::quaternionTFToMsg(tmp.R,tmp.Rs);
    //ROS_INFO("[1]  [quaternionTFToMsg] [%lf %lf %lf %lf]",tmp.Rs.x,tmp.Rs.y,tmp.Rs.z,tmp.Rs.w);
    
}
inline void msg2tf(dataType& tmp)
{
    tmp.P=tf::Vector3(tmp.Ps.x,tmp.Ps.y,tmp.Ps.z);
    tmp.AP=tf::Vector3(tmp.AgPs.x,tmp.AgPs.y,tmp.AgPs.z);
    tmp.AV=tf::Vector3(tmp.AgVs.x,tmp.AgVs.y,tmp.AgVs.z);
    tmp.V=tf::Vector3(tmp.Vs.x,tmp.Vs.y,tmp.Vs.z);
    tmp.Ba=tf::Vector3(tmp.Bas.x,tmp.Bas.y,tmp.Bas.z);
    tmp.Bg=tf::Vector3(tmp.Bgs.x,tmp.Bgs.y,tmp.Bgs.z);
    //ROS_INFO("[2 before ]  [quaternionMsgToTF] [%lf %lf %lf %lf]",tmp.Rs.x,tmp.Rs.y,tmp.Rs.z,tmp.Rs.w);
    tf::quaternionMsgToTF(tmp.Rs,tmp.R);
    tmp.R.normalize();
    //ROS_INFO("[2 after]  [quaternionMsgToTF] [%lf %lf %lf %lf]",tmp.R.getX(),tmp.R.getY(),tmp.R.getZ(),tmp.R.getW());
}
dataType interpolation(dataType s,dataType e,double time)
{
    dataType tmp;
    tmp.pix=s.pix;
    tmp.P=s.P+((e.P-s.P)/(e.header-s.header))*(time-s.header);
    tmp.AP=s.AP+((e.AP-s.AP)/(e.header-s.header))*(time-s.header);

    tmp.V=s.V+((e.V-s.V)/(e.header-s.header))*(time-s.header);
    tmp.AV=s.AV+((e.AV-s.AV)/(e.header-s.header))*(time-s.header);
    tmp.Ba=s.Ba+((e.Ba-s.Ba)/(e.header-s.header))*(time-s.header);
    tmp.Bg=s.Bg+((e.Bg-s.Bg)/(e.header-s.header))*(time-s.header);
    tf::Quaternion q1=s.R; 
    tf::Quaternion q2=e.R; 
    tf::Quaternion q3=q1.slerp(q2,(time-s.header)/(e.header-s.header));
    q3.normalize();
    tmp.R=q3;
    tf2msg(tmp);
    return tmp;
}
dataType pred_interpolation(dataType s,double time)
{
    dataType tmp;
    tmp.V=s.V;
    tmp.P=s.P+s.V*(time-s.header);
    tmp.AV=s.AV;
    tmp.AP=s.AP+s.AV*(time-s.header);
    tmp.Ba=s.Ba;
    tmp.Bg=s.Bg;
    tmp.R=s.R;
    tmp.R.normalize();
    tmp.pix=s.pix;
    tf2msg(tmp);
    return tmp;
}
bool reqDataFromServer(vins::reqdata::Request &req, vins::reqdata::Response &res){
    double randomCheckSeed=req.seed;
    m_buf.lock();
    //int ag=req.agent;
    int comp=req.comp;
    int num=req.num;
    //ROS_INFO("%d %d req data begin\n",comp,num);
    for(int i=0;i<num;i++){
        res.status[i]=127;
        int belong=-1;
        double time=req.header[i];
        if(data[comp].size()<=1){
            res.status[i]=101;
            continue;
        }
        auto iter = data[comp].begin(); 
        iter=data[comp].lower_bound(time);
        if(iter!=data[comp].end() && abs(iter->first-time)>2.0){
            res.status[i]=102;
            continue;
        }
        dataType interpolation_obj;
        if(iter!=data[comp].end()&&abs(time-iter->first)<=0.0002)
        {
            interpolation_obj=iter->second;
            belong=1;
            //ROS_INFO("info 200  %d %lf %lf %lf %lf %lf %lf",i,interpolation_obj.AgPs.y,interpolation_obj.AgPs.x,interpolation_obj.Rs.z,
            //interpolation_obj.AP.getX(),interpolation_obj.AP.getY(),interpolation_obj.AP.getZ());
        }
        else{

            if(iter==data[comp].end()){
                iter=std::prev(iter);
                //auto pl=data[comp].rbegin();
                if(abs(iter->first-time)>0.1){

                    //ROS_INFO("error 103 ---- %lf %lf %lf",time,iter->first,data[comp].rbegin()->first);
                    res.status[i]=103;
                    continue;
                }
                else{
                    interpolation_obj=pred_interpolation(iter->second,time);
                    belong=4;
                }
            }
            else if(iter==data[comp].begin()){
                if(abs(iter->first-time)>0.1){

                    //ROS_INFO("error 104 ---- %lf %lf %lf",time,iter->first,data[comp].begin()->first);
                    res.status[i]=104;
                    continue;
                }

                
                interpolation_obj=pred_interpolation(iter->second,time);
                belong=2;
                //ROS_INFO("info 105   %lf %lf %lf %lf %lf %lf",interpolation_obj.AgPs.y,interpolation_obj.AgPs.x,interpolation_obj.Rs.x,
                //interpolation_obj.AP.getX(),interpolation_obj.AP.getY(),interpolation_obj.AP.getZ());
            }
            else{
                auto prev_iter=std::prev(iter);
                belong=3;
                interpolation_obj=interpolation( prev_iter->second, iter->second,time);
                //ROS_INFO("info 10666   %d  %lf %lf %lf %lf %lf %lf %lf %lf",
                ////i,interpolation_obj.AgPs.y,interpolation_obj.AgPs.x,interpolation_obj.Rs.y,interpolation_obj.AgPs.z,iter->first,
                //interpolation_obj.AP.getX(),interpolation_obj.AP.getY(),interpolation_obj.AP.getZ());
            }
        }
        auto iter2 = cent_ang_map.begin();
        iter2=cent_ang_map.lower_bound(time);
        int flag=1;
        if(iter2==cent_ang_map.end() || iter2->first-time>0.03){
            res.Wstatus[i]=104;
            //continue;
            flag=0;
        }
        if(iter2==cent_ang_map.begin()){
            res.Wstatus[i]=104;
            //continue;
            flag=0;
        }

        auto iter3=cent_imu_map.begin();
        iter3=cent_imu_map.lower_bound(time);

        if(iter3==cent_imu_map.end()|| iter3->first-time>0.03){

            //ROS_INFO("109   %lf %lf",iter3->first,time);
            res.Wstatus[i]=109;
            flag=0;
        }
        if(flag==1)
        {
            auto prev_iter2=std::prev(iter2);
            Vector3d dis=prev_iter2->second+((iter2->second-prev_iter2->second)/(iter2->first-prev_iter2->first)*(time-prev_iter2->first));
            res.Ws[i].x=dis(0);
            res.Ws[i].y=dis(1);
            res.Ws[i].z=dis(2);
            res.Wstatus[i]=200;


            auto prev_iter3=std::prev(iter3);
            Eigen::Quaterniond q1=prev_iter3->second;
            Eigen::Quaterniond q2=iter3->second;
            q1=q1.slerp((time-prev_iter3->first)/(iter3->first-prev_iter3->first),q2);
            tf::quaternionEigenToMsg(q1,res.CentRs[i]);
        }
        
        //ROS_INFO("%lf %lf %lf %lf %lf %lf  ------ %lf %lf %lf %lf %lf %lf ",
        //interpolation_obj.Vs.x,interpolation_obj.Vs.y,interpolation_obj.Vs.z,
        //iter->second.Vs.x,iter->second.Vs.y,iter->second.Vs.z ,
        //interpolation_obj.Ps.x,interpolation_obj.Ps.y,interpolation_obj.Ps.z,
        //iter->second.Ps.x,iter->second.Ps.y,iter->second.Ps.z);
        tf2msg(interpolation_obj);
        res.Ps[i]=interpolation_obj.Ps;
        res.Vs[i]=interpolation_obj.Vs;
        res.Rs[i]=interpolation_obj.Rs;
        res.Bas[i]=interpolation_obj.Bas;
        res.Bgs[i]=interpolation_obj.Bgs;
        res.pix[i]=interpolation_obj.pix;
        res.AgPs[i]=interpolation_obj.AgPs;
        res.AgVs[i]=interpolation_obj.AgVs;

        if(abs(res.AgPs[i].x)<0.0001||abs(res.AgPs[i].y)<0.0001||abs(res.AgPs[i].z)<0.0001){
            if(belong!=-1)
            {
                ROS_INFO("find %d seed==%lf info belong=%d   %d  %lf  %lf %lf %lf %lf %lf %lf %lf",
                comp,randomCheckSeed,belong,i,time,res.AgPs[i].x,res.AgPs[i].y,res.AgPs[i].z,res.Rs[i].x,res.Rs[i].y,res.Rs[i].z,res.Rs[i].w);
            }
            else{
                ROS_WARN("don't find %lf in map %lf ---- %lf",time,data[comp].begin()->first,data[comp].rbegin()->first);
            }
            
        }
        if(belong!=-1)res.status[i]=200;
    }
    //ROS_INFO("%d %d req data begin\n",comp,num);
    m_buf.unlock();
    return true;
}
bool subDataToServer(vins::subdata::Request &req, vins::subdata::Response &res){
    

    m_buf.lock();
    
    int agent_id=req.agent;
    int num=req.num;
    //ROS_INFO("%d %d subdata begin\n",agent_id,num);
    ag_Time[agent_id]=ag[agent_id].toc()/1000;
    //ROS_INFO("receive [num===%d]  [agent=%d]",num,agent_id);
    for(int i=0;i<num;i++)
    {
        dataType tmp;
        tmp.header=req.header[i];
        tmp.Ps=req.Ps[i];
        tmp.Vs=req.Vs[i];
        tmp.Rs=req.Rs[i];
        tmp.Bas=req.Bas[i];
        tmp.Bgs=req.Bgs[i];
        tmp.pix=req.pix;
        tmp.AgPs=req.AgPs[i];
        tmp.AgVs=req.AgVs[i];
        //ROS_INFO("tmp.AgPs %d %d  %lf %lf %lf   %lf",agent_id,i,tmp.Rs.x,tmp.Rs.y,tmp.Rs.z,tmp.Rs.w);
        msg2tf(tmp);
        if(abs(tmp.AgPs.x)<=0.00001){
            ROS_WARN("subbbb   %d  %lf  %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            i,tmp.header,tmp.AgPs.x,tmp.AgPs.y,tmp.AgPs.z,tmp.Rs.x,tmp.Rs.y,tmp.Rs.z,tmp.AP.getX(),tmp.AP.getY(),tmp.AP.getZ());
        }
        data[agent_id][tmp.header]=tmp;
    }
    //ROS_INFO("%d ",res.status);
    res.status=200;
    //ROS_INFO("%d %d subdata end\n",agent_id,num);
    m_buf.unlock();
    return true;
}
bool reqTimeFromServer(vins::timedata::Request &req, vins::timedata::Response &res){
    
    m_buf.lock();
    double time=192608170000;
    // for(int i=1;i<=3;i++){
    //     if(data[i].size()==0)
    //     cout<<i<<"  "<<0<<"  ";
    //     else
    //     cout<<i<<"  "<<data[i].rbegin()->first<<" ";
    // }
    //cout<<endl;
    for(int i=1;i<=3;i++){
        if(i==req.agent)continue;
        if(data[i].size()==0){
            time=0;
            break;
        }
        if(data[i].size()>=1)
        {
            //ROS_INFO("time req %lf %lf ",ag_Time[i],ag[i].toc()/1000);
            if(ag_Time[i]<ag[i].toc()/1000-17.5){
                continue;
            }
            time=min(time,data[i].rbegin()->first);
        }
        
    }
    res.status=200;
    res.time=time;
    m_buf.unlock();
    return true;
}
void center_callback(const sensor_msgs::ImuConstPtr &imu_msg){
    
    m_buf.lock();
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    double ax = imu_msg->linear_acceleration.x;
    double ay = imu_msg->linear_acceleration.y;
    double az = imu_msg->linear_acceleration.z;
    tf::Quaternion Q;
    tf::quaternionMsgToTF(imu_msg->orientation,Q);
    Q.normalize();
    Eigen::Quaterniond q_odom_curr_tmp;
    q_odom_curr_tmp.x()=Q.getX();
    q_odom_curr_tmp.y()=Q.getY();
    q_odom_curr_tmp.z()=Q.getZ();
    q_odom_curr_tmp.w()=Q.getW();
    q_odom_curr_tmp.normalize();

    if(sim_flag==1){
        Eigen::Vector3d delta(qu(qe)/3.1415*180,qu(qe)/3.1415*180,qu(qe)/3.1415*180);
        Eigen::Matrix3d deltaMat=Utility::ypr2R(delta);
        q_odom_curr_tmp=Eigen::Quaterniond{deltaMat*q_odom_curr_tmp.toRotationMatrix()};
        q_odom_curr_tmp.normalize();
    }

    cent_imu_map[imu_msg->header.stamp.toSec()]=q_odom_curr_tmp;
    q_cent_pose.push(make_pair(imu_msg->header,q_odom_curr_tmp));
    //Matrix3d R0=Utility::ypr2R(Eigen::Vector3d{-170, -2, -7});
    Matrix3d R0;
    Eigen::Vector3d ws(rx,ry,rz);
    Vector3d gyr(rx, ry, rz);
    Vector3d acc(ax,ay,az);
    gyr=q_odom_curr_tmp*gyr;
    //gyr(2)*=-1;
    cent_ang_map[imu_msg->header.stamp.toSec()]=gyr;
    cent_acc_map[imu_msg->header.stamp.toSec()]=acc;
    m_buf.unlock();
}
struct est_cent_imu_gt_factor
{
    est_cent_imu_gt_factor(const Eigen::Quaterniond _q1,const Eigen::Quaterniond _q2){
        q1=_q1;
        q2=_q2;
    }
    template <typename T>
    bool operator()(const T* rt, T* residuals) const{
        
          Eigen::Quaternion<T> Qi{(T)q1.w(),(T)q1.x(),(T)q1.y(),(T)q1.z()};
          Eigen::Quaternion<T> Qj{(T)q2.w(),(T)q2.x(),(T)q2.y(),(T)q2.z()};
          Eigen::Quaternion<T> Qk{(T)rt[6],(T)rt[3],(T)rt[4],(T)rt[5]};
          Eigen::Quaternion<T> Q_est=Qk*Qi;
          Eigen::Quaternion<T> Q_error=Q_est.inverse()*Qj;
          Q_error.normalize();
          residuals[0]=Q_error.x();
          residuals[1]=Q_error.y();
          residuals[2]=Q_error.z();
          residuals[3]=Q_error.w()-(T)1;
          for(int i=0;i<4;i++)
          residuals[i]*=(T)100;
          return true;
    }
    Eigen::Quaterniond q1,q2;
};
void est_cent_imu_gt()
{
    int baseOffet=1100;
    int endOffet=3100;
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //double para_cent_imu_gt[7]={0,0,0,-0.7071068, 0, 0, 0.7071068};
    double para_cent_imu_gt[7]={0,0,0,0, 0, 0, 1};
    loss_function = new ceres::HuberLoss(5.0);
    
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_cent_imu_gt, 7,local_parameterization);
    
    Eigen:: Vector3d euler(0,0,0);
    ROS_INFO("[%lf] [%lf] [%lf]",euler.x(),euler.y(),euler.z());
    auto it=cent_imu_map.begin();
    for(int i=0;i<baseOffet;i++)it++;
    int cnt=0;

    Eigen::Quaterniond q_base(1,0,0,0);
    for(int i=baseOffet;i<=endOffet;i+=1)
    {
        double time=it->first;
        it++;
        auto it2=cent_gt_map.lower_bound(time);
        
        if(it2!=cent_gt_map.end() && it2!=cent_gt_map.begin()&&abs(it2->first-time)<0.015)
        {
            
            auto prev_iter=std::prev(it2);
            Eigen::Quaterniond q1=prev_iter->second;
            Eigen::Quaterniond q2=it2->second;
            Eigen::Quaterniond q=q1.slerp((time-prev_iter->first)/(it2->first-prev_iter->first),q2);
            q.normalize();
            it->second.normalize();
            Eigen::Quaterniond q_update=q*it->second.inverse();
            q_base=q_base.slerp(0.5,q_update);
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<est_cent_imu_gt_factor,4,7>(new est_cent_imu_gt_factor(it->second,q)),
                                loss_function,para_cent_imu_gt
            );
            cnt++;
            Eigen:: Matrix3d tmp;
            Eigen:: Matrix3d r1=it->second.toRotationMatrix(),r2=q.toRotationMatrix();
            tmp=r2*r1.inverse();
            euler+=Utility::R2ypr(tmp);
        }
    }
    euler/=cnt;
    Eigen::Quaterniond q_init(Utility::ypr2R(euler));
    para_cent_imu_gt[6]=q_base.w();
    para_cent_imu_gt[3]=q_base.x();
    para_cent_imu_gt[4]=q_base.y();
    para_cent_imu_gt[5]=q_base.z();
    ROS_INFO("[%lf] [%lf] [%lf]",euler.x(),euler.y(),euler.z());
    ceres::Solver::Options options;
                //options.disable_all_safety_checks=false;
    //options.linear_solver_type = ceres::DENSE_QR;
    options.num_threads = 4;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations=60;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.final_cost <<"\n";
    std::cout << summary.BriefReport() << "\n";
    for(int i=0;i<=6;i++)std::cout<<para_cent_imu_gt[i]<<" ";
    std::cout<<endl;
    printf("%.2lf\n",1.555);
    Q_imu_to_gt=Eigen::Quaterniond(para_cent_imu_gt[6],para_cent_imu_gt[3],para_cent_imu_gt[4],para_cent_imu_gt[5]).normalized();
    R_imu_to_gt=Eigen::Quaterniond(para_cent_imu_gt[6],para_cent_imu_gt[3],para_cent_imu_gt[4],para_cent_imu_gt[5]).normalized().toRotationMatrix();
    for(int i=0;i<3;i++){
        printf("%.5lf %.5lf %.5lf\n",R_imu_to_gt(i,0),R_imu_to_gt(i,1),R_imu_to_gt(i,2));
    }
    flag_imu_to_gt=true;
}
void center_ground_pose_callback(const geometry_msgs::PoseStampedPtr &posePtr)
{
    m_buf.lock();
    double time=posePtr->header.stamp.toSec();
    Eigen::Quaterniond q_odom_curr_tmp;
    tf::quaternionMsgToEigen(posePtr->pose.orientation,q_odom_curr_tmp);
    q_odom_curr_tmp.normalized();
    cent_gt_map[time]=q_odom_curr_tmp;
    if(cent_gt_map.size()>=2000&&cent_imu_map.size()>=3500&&flag_imu_to_gt==false){
        est_cent_imu_gt();
    }
    m_buf.unlock();
}
bool accDataFromServer(vins::accdata::Request &req, vins::accdata::Response &res){
    m_buf.lock();
    //int ag=req.agent;
    int num=req.num;
    for(int i=0;i<num;i++){
        double time=req.header[i];
        auto iter = cent_acc_map.begin();
        iter=cent_acc_map.lower_bound(time);
        if(iter==cent_acc_map.end() || iter->first-time>1){
            res.status[i]=101;
            continue;
        }
        if(iter==cent_acc_map.begin()){
            res.status[i]=101;
            continue;
        }
        auto prev_iter=std::prev(iter);
        Vector3d dis=prev_iter->second+((iter->second-prev_iter->second)/(iter->first-prev_iter->first)*(time-prev_iter->first));
        res.status[i]=200;
        res.As[i].x=dis(0);
        res.As[i].y=dis(1);
        res.As[i].z=dis(2);
    }
    m_buf.unlock();
    return true;
}


void groundPose_callback_1(const geometry_msgs::PoseStampedPtr &posePtr)
{
    m_buf.lock();
    //gt[1][posePtr->header.stamp.toSec()]=*posePtr;
    //dataType tmp;
    Eigen::Vector3d pos;
    tf::pointMsgToEigen(posePtr->pose.position,pos);
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(posePtr->pose.orientation,q);
    q.normalize();
    //pos+=q.toRotationMatrix()*Eigen::Vector3d(0,0,0);
    UWBType m;
    for(int i=0;i<=4;i++)
    {
        m.dis[i]=(pos-AnchorPos[i]).norm()+u(e);
        // if(big_uwb_noise_number[1]>0&&big_uwb_noise){
        //     m.dis[i]+=cu(ce);
        //     big_uwb_noise_number[1]--;
        // }
        // else{
        //     double rs=cu(ce);
        //     if(abs(rs)>0.3*1&&big_uwb_noise){
        //         big_uwb_noise_number[1]=300;
        //         m.dis[i]+=cu(ce);
        //     }
        // }
        
    }
    m.ps=*posePtr;
    gt[1][posePtr->header.stamp.toSec()]=m;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped=*posePtr;
    no_path_o[1].header = pose_stamped.header;
    no_path_o[1].header.frame_id = "world";
    no_path_o[1].poses.push_back(pose_stamped);
    no_pub_path[1].publish(no_path_o[1]);
    geometry_msgs::Vector3 vc;
    vc.x=m.dis[0];
    vc.y=m.dis[1];
    vc.z=m.dis[2];
    uwb_mean_data[1].publish(vc);
    m_buf.unlock();
    
}
void groundPose_callback_2(const geometry_msgs::PoseStampedPtr &posePtr)
{
    
    m_buf.lock();
    Eigen::Vector3d pos;
    tf::pointMsgToEigen(posePtr->pose.position,pos);
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(posePtr->pose.orientation,q);
    q.normalize();
    //pos+=q.toRotationMatrix()*Eigen::Vector3d(0,0,0);
    UWBType m;
    for(int i=0;i<=4;i++)
    {
        m.dis[i]=(pos-AnchorPos[i]).norm()+u(e);
        // if(big_uwb_noise_number[2]>0){
        //     m.dis[i]+=cu(ce);
        //     big_uwb_noise_number[2]--;
        // }
        // else{
        //     double rs=cu(ce);
        //     if(abs(rs)>0.3*1){
        //         big_uwb_noise_number[2]=300;
        //         m.dis[i]+=cu(ce)*1.25;
        //     }
        // }
    }
    m.ps=*posePtr;
    gt[2][posePtr->header.stamp.toSec()]=m;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped=*posePtr;
    no_path_o[2].header = pose_stamped.header;
    no_path_o[2].header.frame_id = "world";
    no_path_o[2].poses.push_back(pose_stamped);
    no_pub_path[2].publish(no_path_o[2]);
    geometry_msgs::Vector3 vc;
    vc.x=m.dis[0];
    vc.y=m.dis[1];
    vc.z=m.dis[2];
    uwb_mean_data[2].publish(vc);
    m_buf.unlock();
}
void groundPose_callback_3(const geometry_msgs::PoseStampedPtr &posePtr)
{
    m_buf.lock();
    Eigen::Vector3d pos;
    tf::pointMsgToEigen(posePtr->pose.position,pos);
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(posePtr->pose.orientation,q);
    q.normalize();
    //pos+=q.toRotationMatrix()*Eigen::Vector3d(0,0,0);
    UWBType m;
    for(int i=0;i<=4;i++)
    {
        m.dis[i]=(pos-AnchorPos[i]).norm()+u(e);
        // if(big_uwb_noise_number[3]>0){
        //     m.dis[i]+=cu(ce);
        //     big_uwb_noise_number[3]--;
        // }
        // else{
        //     double rs=cu(ce);
        //     if(abs(rs)>0.3*1){
        //         big_uwb_noise_number[3]=300;
        //         m.dis[i]+=cu(ce)*1.25;
        //     }
        // }
    }
    m.ps=*posePtr;
    gt[3][posePtr->header.stamp.toSec()]=m;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped=*posePtr;
    no_path_o[3].header = pose_stamped.header;
    no_path_o[3].header.frame_id = "world";
    no_path_o[3].poses.push_back(pose_stamped);
    no_pub_path[3].publish(no_path_o[3]);
    geometry_msgs::Vector3 vc;
    vc.x=m.dis[0];
    vc.y=m.dis[1];
    vc.z=m.dis[2];
    uwb_mean_data[3].publish(vc);
    m_buf.unlock();
}



void groundOdometry_callback_1(const nav_msgs::OdometryConstPtr &posePtr)
{
    m_buf.lock();
    
    Eigen::Vector3d pos;
    tf::pointMsgToEigen(posePtr->pose.pose.position,pos);
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(posePtr->pose.pose.orientation,q);
    q.normalize();
    UWBType m;
    for(int i=0;i<=4;i++)
    {
        m.dis[i]=(pos-AnchorPos[i]).norm()+u(e);
    }
    m.ps.pose.position=posePtr->pose.pose.position;
    m.ps.pose.orientation=posePtr->pose.pose.orientation;
    gt[1][posePtr->header.stamp.toSec()]=m;
    m_buf.unlock();
    
}
void groundOdometry_callback_2(const nav_msgs::OdometryConstPtr &posePtr)
{
    m_buf.lock();
    
    Eigen::Vector3d pos;
    tf::pointMsgToEigen(posePtr->pose.pose.position,pos);
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(posePtr->pose.pose.orientation,q);
    q.normalize();
    UWBType m;
    for(int i=0;i<=4;i++)
    {
        m.dis[i]=(pos-AnchorPos[i]).norm()+u(e);
    }
    m.ps.pose.position=posePtr->pose.pose.position;
    m.ps.pose.orientation=posePtr->pose.pose.orientation;
    gt[2][posePtr->header.stamp.toSec()]=m;
    m_buf.unlock();
    
}
void groundOdometry_callback_3(const nav_msgs::OdometryConstPtr &posePtr)
{
    m_buf.lock();
    
    Eigen::Vector3d pos;
    tf::pointMsgToEigen(posePtr->pose.pose.position,pos);
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(posePtr->pose.pose.orientation,q);
    q.normalize();
    UWBType m;
    for(int i=0;i<=4;i++)
    {
        m.dis[i]=(pos-AnchorPos[i]).norm()+u(e);
    }
    m.ps.pose.position=posePtr->pose.pose.position;
    m.ps.pose.orientation=posePtr->pose.pose.orientation;
    gt[3][posePtr->header.stamp.toSec()]=m;
    m_buf.unlock();
    
}
bool reqWrpDataServer(vins::wrpdata::Request &req, vins::wrpdata::Response &res){
    m_buf.lock();
    int ag=req.agent;
    double time=req.header;
    res.status=101;
    auto iter=gt[ag].lower_bound(time);
    if(iter!=gt[ag].end() && abs(time-iter->first)<0.04){

        auto pt=std::prev(iter);
        geometry_msgs::PoseStamped pose=iter->second.ps;
        geometry_msgs::PoseStamped lastpose=pt->second.ps;

        Eigen::Vector3d x1(
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z
        ),x2(lastpose.pose.position.x,lastpose.pose.position.y,lastpose.pose.position.z);
        //tf::vectorMsgToEigen(pose.pose.position,x1);
        //tf::vectorMsgToEigen(lastpose.pose.position,x2);
        x1=(x1-x2)*(time-pt->first)/(iter->first-pt->first)+x2;
        tf::vectorEigenToMsg(x1,res.Ps);
        Eigen::Quaterniond q_odom_curr_tmp,q_last;
        tf::quaternionMsgToEigen(pose.pose.orientation,q_odom_curr_tmp);
        tf::quaternionMsgToEigen(lastpose.pose.orientation,q_last);
        q_odom_curr_tmp=q_last.slerp((time-pt->first)/(iter->first-pt->first),q_odom_curr_tmp);

        q_odom_curr_tmp.normalize();
        tf::quaternionEigenToMsg(q_odom_curr_tmp,res.Rs);        
        for(int i=0;i<=4;i++){
            res.dis[i]=(iter->second.dis[i]-pt->second.dis[i])*(time-pt->first)/(iter->first-pt->first)+pt->second.dis[i];
        }
        //ROS_INFO("%d %lf %d 200 ",ag,time,gt[ag].size());
        res.status=200;
    }
    else{
        //ROS_INFO("%d %lf %d 101 ",ag,time,gt[ag].size());
        res.status=101;
    }
    
    m_buf.unlock();
    return true;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "multi_server");
    ros::NodeHandle nh;

    for(int i=0;i<=3;i++){
        std::map<double,dataType> tmp;
        data[i]=tmp;

    }
    for(int i=1;i<=3;i++){
        ag[i].tic();
        ag_Time[i]=ag[i].toc()/1000;
    }
    //vrpn_client_node
    ros::Subscriber gt_calib_switch_1;
    ros::Subscriber gt_calib_switch_2;
    ros::Subscriber gt_calib_switch_3;
    if(sim_flag==0){
        gt_calib_switch_1 = nh.subscribe("/vrpn_client_node/robot12/pose", 200, groundOdometry_callback_1);
        gt_calib_switch_2 = nh.subscribe("/vrpn_client_node/robot22/pose", 200, groundOdometry_callback_2);
        gt_calib_switch_3 = nh.subscribe("/vrpn_client_node/robot32/pose", 200, groundOdometry_callback_3);
    }
    else{
        gt_calib_switch_1 = nh.subscribe("/pose_1", 200, groundOdometry_callback_1);
        gt_calib_switch_2 = nh.subscribe("/pose_3", 200, groundOdometry_callback_2);
        gt_calib_switch_3 = nh.subscribe("/pose_2", 200, groundOdometry_callback_3);
    }
    ros::ServiceServer sub_data_server = nh.advertiseService("/subDataToServer",subDataToServer);
    ros::Subscriber centerAngular;
    if(sim_flag==0)centerAngular = nh.subscribe("/mavros/imu/data", 2000, center_callback);
    else centerAngular = nh.subscribe("/imu_no_noise_0", 2000, center_callback);
    //ros::Subscriber center_true_value = nh.subscribe("/vrpn_client_node/ag0312/pose", 2000, center_ground_pose_callback);
    ros::ServiceServer req_data_server = nh.advertiseService("/reqDataFromServer",reqDataFromServer);
    ros::ServiceServer req_time_server = nh.advertiseService("/reqTimeFromServer",reqTimeFromServer);
    ros::ServiceServer acc_time_server = nh.advertiseService("/accDataFromServer",accDataFromServer);
    ros::ServiceServer req_wrp_data_server = nh.advertiseService("/reqWrpDataServer",reqWrpDataServer);
    no_pub_path[1] = nh.advertise<nav_msgs::Path>("/vins_1/vins_estimator/gt", 1000);
    no_pub_path[2] = nh.advertise<nav_msgs::Path>("/vins_2/vins_estimator/gt", 1000);
    no_pub_path[3] = nh.advertise<nav_msgs::Path>("/vins_3/vins_estimator/gt", 1000);
    uwb_mean_data[1] = nh.advertise<geometry_msgs::Vector3>("/vins_1/uwb", 1000);
    uwb_mean_data[2] = nh.advertise<geometry_msgs::Vector3>("/vins_2/uwb", 1000);
    uwb_mean_data[3] = nh.advertise<geometry_msgs::Vector3>("/vins_3/uwb", 1000);
    ang_calib=nh.advertise<nav_msgs::Odometry>("/ang_calib", 500);
    //ros::ServiceServer ang_time_server = nh.advertiseService("/angDataFromServer",angDataFromServer);
    ROS_INFO("waiting two numbers...");
    ros::spin();
    //calcTimeDelta();
    return 0;
}
