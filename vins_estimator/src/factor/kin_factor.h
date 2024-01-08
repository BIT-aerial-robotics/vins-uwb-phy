#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"
template <typename T>
Eigen::Matrix<T,3,3> fromYawToMat(T yaw)
{
    Eigen::Matrix<T,3,3> mat;
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)mat(i,j)=(T)0;
    yaw = yaw / T(180.0) * T(M_PI);
    mat(0,0)=cos(yaw);
    mat(0,1)=-sin(yaw);
    mat(1,0)=sin(yaw);
    mat(1,1)=cos(yaw);
    mat(2,2)=(T)(1);
    return mat;
}
template <typename T>
Eigen::Matrix<T,3,3> skewSymmetricTemplate(Eigen::Matrix<T,3,1> w)
{
  Eigen::Matrix<T,3,3> ans;
  ans << T(0), -w(2), w(1),
            w(2), T(0), -w(0),
            -w(1), w(0), T(0);
  return ans;
}

struct kinFactor_bet_4dof_2
{
  kinFactor_bet_4dof_2(double POSEI[],double YawI[], double POSEJ[],double YawJ[],double _deltaTime, Eigen::Matrix<double,4,1> _info)
  {
    di=Eigen::Vector3d(POSEI[0],POSEI[1],POSEI[2]);
    dj=Eigen::Vector3d(POSEJ[0],POSEJ[1],POSEJ[2]);
    yi=YawI[0];
    yj=YawJ[0];
    deltay=yj-yi;
    delta_d=dj-di;
    deltaTime=_deltaTime;
    info=_info;
  }
  template <typename T>
  bool operator()(const T*  pi,const T* yi,const T*  pj,const T* yj,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Pj,Vi,Vj;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Pj(i)=pj[i];
    }
    //cout<<delta_d.x()<<" "<<delta_d.y()<<" "<<delta_d.z()<<std::endl;
    Eigen::Matrix<T, 3, 1> pre_delta_p((T)delta_d.x(),(T)delta_d.y(),(T)delta_d.z());//= dj-di;
    Eigen::Matrix<T, 3, 1> aft_delta_p = Pj-Pi;
    aft_delta_p-=pre_delta_p;
    T aft_delta_yaw=yj[0]-yi[0];
    T pre_delta_yaw=(T)deltay;
    aft_delta_yaw-=pre_delta_yaw;
    residuals[0]=aft_delta_p(0);
    residuals[1]=aft_delta_p(1);
    residuals[2]=aft_delta_p(2);
    residuals[3]=aft_delta_yaw;
    for(int i=0;i<=3;i++)
    {
      residuals[i]=residuals[i]*(T)(1/(info(i)));
    }
    return true;
  }
  Eigen::Vector3d di,dj,vi,vj,delta_d,delta_v;
  double yi,yj,deltay;
  double pixi;
  double pixj;
  int Two;
  double deltaTime;
  Eigen::Matrix<double,4,1> info;
};

struct kinFactor_connect_4dof
{
  kinFactor_connect_4dof(double POSEI[], double POSEJ[],double para_hinge,double _info)
  {
    di=Eigen::Vector3d(POSEI[0],POSEI[1],POSEI[2]);
    dj=Eigen::Vector3d(POSEJ[0],POSEJ[1],POSEJ[2]);
    
    ri=Eigen::Quaterniond(POSEI[6],POSEI[3],POSEI[4],POSEI[5]);
    rj=Eigen::Quaterniond(POSEJ[6],POSEJ[3],POSEJ[4],POSEJ[5]);
    
    di+=ri.toRotationMatrix()*(Eigen::Vector3d(0,0,para_hinge));
    dj+=rj.toRotationMatrix()*(Eigen::Vector3d(0,0,para_hinge));
    info=_info;
  }
  template <typename T>
  bool operator()(const T*  pi,const T*  pj,const T* yi,const T* yj,const T* length,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Pj,Vi,Vj,Di,Dj;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Pj(i)=pj[i];
        Di(i)=(T)di(i);
        Dj(i)=(T)dj(i);
    }
    Eigen::Matrix<T, 3, 3>Yawi=fromYawToMat(yi[0]);
    Eigen::Matrix<T, 3, 3>Yawj=fromYawToMat(yj[0]);
    T len=length[0];
    Pi=Yawi*Di+Pi;
    Pj=Yawj*Dj+Pj;
    Eigen::Matrix<T,3,1>dis=Pi-Pj;
    T est_len=dis.norm();
    residuals[0]=(est_len-len)/(T(info));
    return true;
  }
  Eigen::Quaterniond ri,rj,delta_r;
  Eigen::Vector3d di,dj,vi,vj,delta_d,delta_v;
  double pixi;
  double pixj;
  int Two;
  double deltaTime;
  double info;
};

struct kinFactor_connect_hyp_4dof
{
  kinFactor_connect_hyp_4dof(double POSEI[], double POSEJ[],double POSEK[],double para_hinge,double _val,double _info)
  {
    di=Eigen::Vector3d(POSEI[0],POSEI[1],POSEI[2]);
    dj=Eigen::Vector3d(POSEJ[0],POSEJ[1],POSEJ[2]);
    dk=Eigen::Vector3d(POSEK[0],POSEK[1],POSEK[2]);
    ri=Eigen::Quaterniond(POSEI[6],POSEI[3],POSEI[4],POSEI[5]);
    rj=Eigen::Quaterniond(POSEJ[6],POSEJ[3],POSEJ[4],POSEJ[5]);
    rk=Eigen::Quaterniond(POSEK[6],POSEK[3],POSEK[4],POSEK[5]);
    di+=ri.toRotationMatrix()*(Eigen::Vector3d(0,0,para_hinge));
    dj+=rj.toRotationMatrix()*(Eigen::Vector3d(0,0,para_hinge));
    dk+=rk.toRotationMatrix()*(Eigen::Vector3d(0,0,para_hinge));
    info=_info;
    z_val=_val;
  }
  template <typename T>
  bool operator()(const T*  pi,const T*  pj,const T*  pk,const T* yi,const T* yj,const T* yk,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Pj,Pk,Di,Dj,Dk;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Pj(i)=pj[i];
        Pk(i)=pk[i];
        Di(i)=(T)di(i);
        Dj(i)=(T)dj(i);
        Dk(i)=(T)dk(i);
    }
    Eigen::Matrix<T, 3, 3>Yawi=fromYawToMat(yi[0]);
    Eigen::Matrix<T, 3, 3>Yawj=fromYawToMat(yj[0]);
    Eigen::Matrix<T, 3, 3>Yawk=fromYawToMat(yk[0]);

    Pi=Yawi*Di+Pi;
    Pj=Yawj*Dj+Pj;
    Pk=Yawk*Dk+Pk;
    Eigen::Matrix<T, 3, 1> P_ji = Pi-Pj;
    Eigen::Matrix<T, 3, 1> P_ki = Pi-Pk;
    
    Eigen::Matrix<T, 3, 1> N=P_ji.cross(P_ki);
    N/=N.norm();
    //std::cout<<N(0)<<"  "<<N(1)<<"  "<<N(2)<<std::endl;
    N(2)-=(T)z_val;
    residuals[0]=N(2)/(T(info));
    //cout<<N(2,0)<<endl;
    return true;
  }
  void print()
  {
    ROS_INFO("print() %d %lf %lf %lf %lf %lf %lf",AGENT_NUMBER,dj.x(),dj.y(),dj.z(),vj.x(),vj.y(),vj.z());
  }
  Eigen::Quaterniond ri,rj,delta_r,rk;
  Eigen::Vector3d di,dj,vi,vj,delta_d,delta_v,dk;
  double pixi;
  double pixj;
  int Two;
  double deltaTime;
  double info;
  double z_val;
};
