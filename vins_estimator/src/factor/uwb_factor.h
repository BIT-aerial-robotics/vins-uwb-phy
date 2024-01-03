#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

struct UWBBiasFactor
{
    UWBBiasFactor(double _bias,double _info)
    {
        old_bias=_bias;
        info=_info;
    }
    template <typename T>
    bool operator()(const T* b,T* residuals) const
    {
        T res=b[0]-(T)old_bias;
        residuals[0]=res/(T)info;
        return true;
    }
    double old_bias;
    double info;
};
struct UwbFactor
{
  UwbFactor(Eigen::Vector3d _wp_1, Eigen::Matrix3d _wr_1,double _distance,double _info)
  {
    wp[0]=_wp_1;
    wr[0]=_wr_1;
    info=_info;
    distance=_distance;
  }
  template <typename T>
  bool operator()(const T*  pi,const T* pj,const T* b,T* residuals) const
  {
    
    Eigen::Matrix<T, 3, 1> Pi[2],WP[2],OT,dis;
    for(int i=0;i<3;i++)
    {
        Pi[0](i)=pi[i];
        Pi[1](i)=pj[i];
        WP[0](i)=(T)wp[0](i);
    }
    Eigen::Matrix<T,3,3> WR[2];
    for(int i=0;i<=2;i++)for(int j=0;j<=2;j++)WR[0](i,j)=(T)wr[0](i,j);
    Eigen::Quaternion<T> Qi(pi[6],pi[3],pi[4],pi[5]);
    //Pi[0]+=Qi.toRotationMatrix()*Eigen::Matrix<T, 3, 1>((T)0,(T)0,(T)0);
    Pi[0]=WR[0]*Pi[0]+WP[0];
    
    dis=Pi[1]-Pi[0];
    
    T est_len=dis.norm();
    T len=(T)distance-b[0];
    residuals[0]=(est_len-len)/(T(info));
    return true;
  }
  Eigen::Vector3d wp[2];
  Eigen::Matrix3d wr[2];
  Eigen::Vector3d ot;
  double distance;
  double deltaTime;
  double info;
};


struct UWBFactor_delta
{
  UWBFactor_delta(Eigen::Vector3d _wp_1, Eigen::Matrix3d _wr_1,Eigen::Vector3d _dp,Eigen::Matrix3d _dr,double _dt,double _distance,double _info)
  {
    wp[0]=_wp_1;
    wr[0]=_wr_1;
    info=_info;
    distance=_distance;
    dp=_dp;
    dr=_dr;
    dt=_dt;
  }
  template <typename T>
  bool operator()(const T*  pi,const T* vi,const T* pj,const T* b,T* residuals) const
  {
    
    Eigen::Matrix<T, 3, 1> Pi[2],WP[2],OT,dis,UP,Vi,Uv;
    for(int i=0;i<3;i++)
    {
        Pi[0](i)=pi[i];
        Pi[1](i)=pj[i];
        WP[0](i)=(T)wp[0](i);
        UP(i)=(T)dp(i);
        Uv(i)=(T)dv(i);
        Vi(i)=(T)vi[i];
    }
    Eigen::Matrix<T,3,3> WR[2],UR;
    for(int i=0;i<=2;i++)for(int j=0;j<=2;j++)WR[0](i,j)=(T)wr[0](i,j),UR(i,j)=(T)dr(i,j);
    Eigen::Quaternion<T> Qi(pi[6],pi[3],pi[4],pi[5]);
    Qi.normalize();
    Pi[0]+=(T)(1.0)*(Qi*UP);
    //Pi[0]+=(T)(0.5)*(Vi*(T)dt);
    Qi=UR*Qi;
    Qi.normalize();
    //Pi[0]+=Qi.toRotationMatrix()*Eigen::Matrix<T, 3, 1>((T)0,(T)0,(T)0.00);
    Pi[0]=WR[0]*Pi[0]+WP[0];
    
    dis=Pi[1]-Pi[0];
    
    T est_len=dis.norm();
    T len=(T)distance-b[0];
    residuals[0]=(est_len-len)/(T(info));
    return true;
  }
  Eigen::Vector3d wp[2],dp;
  Eigen::Matrix3d wr[2],dr;
  Eigen::Vector3d ot;
  Eigen::Vector3d v,dv;
  double distance;
  double deltaTime;
  double info,dt;
};

