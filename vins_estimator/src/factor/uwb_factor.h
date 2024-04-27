#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "../utility/utility.h"
#include "../estimator/parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

struct UWBBiasFactor
{
    UWBBiasFactor(double _bias[],double _info)
    {
        old_bias_1=_bias[0];
        old_bias_2=_bias[1];
        info=_info;
    }
    template <typename T>
    bool operator()(const T* b,T* residuals) const
    {
        T res=b[0]-(T)old_bias_1;
        residuals[0]=res/(T)info;
        res=b[1]-(T)old_bias_2;
        residuals[1]=res/(T)info;
        return true;
    }
    double old_bias_1,old_bias_2;
    double info;
};
struct UWBAnchorFactor
{
    UWBAnchorFactor(double _pos[],double _info)
    {
        for(int i=0;i<=2;i++)pos[i]=_pos[i];
        info=_info;
    }
    template <typename T>
    bool operator()(const T* b,T* residuals) const
    {
        for(int i=0;i<=2;i++){
          T res=b[i]-(T)pos[i];
          residuals[i]=res/(T)info;
        }
        return true;
    }
    double pos[3];
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
  bool operator()(const T*  pi,const T* pj,const T* b,const T*tag,T* residuals) const
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
    Pi[0]=WR[0]*(Pi[0]+Qi*Eigen::Matrix<T,3,1>(tag))+WP[0];
    
    dis=Pi[1]-Pi[0];
    T est_len=dis.norm();
    est_len=est_len*b[1]+b[0];
    T len=(T)distance;
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
  bool operator()(const T*  pi,const T* vi,const T* pj,const T* b,const T* tag,T* residuals) const
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
    T sum_dt=(T)dt;
    Pi[0]+=(T)(1.0)*(Qi*UP)+Vi*sum_dt-0.5*G*sum_dt*sum_dt;
    //Pi[0]+=(T)(0.5)*(Vi*(T)dt);
    Qi=UR*Qi;
    Qi.normalize();
    //Pi[0]+=Qi.toRotationMatrix()*Eigen::Matrix<T, 3, 1>((T)0,(T)0,(T)0.00);
    Pi[0]=WR[0]*(Pi[0]+Qi*Eigen::Matrix<T,3,1>(tag))+WP[0];
    
    dis=Pi[1]-Pi[0];
    
    T est_len=dis.norm();
    est_len=est_len*b[1]+b[0];
    T len=(T)distance;
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


struct UWBFactor_connect_4dof
{
  UWBFactor_connect_4dof(Eigen::Vector3d pi,Eigen::Quaterniond qi,double para_hinge,double _dis,double _info)
  {
    di=pi;
    ri=qi;
    di+=ri.toRotationMatrix()*(Eigen::Vector3d(0,0,para_hinge));
    info=_info;
    dis=_dis;
  }
  template <typename T>
  bool operator()(const T*  pi,const T* yi,const T* pj,const T*bias,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Vi,Di,Pj;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Di(i)=(T)di(i);
        Pj(i)=pj[i];
    }
    Eigen::Matrix<T, 3, 3>Yawi=fromYawToMat(yi[0]);
    T len=(T)dis-bias[0];
    Pi=Yawi*Di+Pi;
    Eigen::Matrix<T,3,1>bet=Pi-Pj;
    T est_len=bet.norm();
    residuals[0]=(est_len-len)/(T(info));
    return true;
  }
  Eigen::Quaterniond ri,rj;
  Eigen::Vector3d di,dj,vi,vj;
  double deltaTime;
  double info,dis;
};


struct UWBFactor_connect_4dof_plus_mul
{
  UWBFactor_connect_4dof_plus_mul(Eigen::Vector3d pi,Eigen::Quaterniond qi,double para_hinge[],double _dis,double _info)
  {
    di=pi;
    ri=qi;
    di+=ri.toRotationMatrix()*(Eigen::Vector3d(para_hinge));
    info=_info;
    dis=_dis;
  }
  template <typename T>
  bool operator()(const T*  pi,const T* yi,const T* pj,const T*bias,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Vi,Di,Pj;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Di(i)=(T)di(i);
        Pj(i)=pj[i];
    }
    Eigen::Matrix<T, 3, 3>Yawi=fromYawToMat(yi[0]);
    T len=(T)dis;
    Pi=Yawi*Di+Pi;
    Eigen::Matrix<T,3,1>bet=Pi-Pj;
    T est_len=bet.norm()*bias[1]+bias[0];
    T logv=(T)log(bias[1])+(T)log(bet.norm())-(T)log(len-bias[0]);
    residuals[0]=(est_len-len)/(T(info));
    //residuals[1]=(logv)/(T(info));
    return true;
  }
  Eigen::Quaterniond ri,rj;
  Eigen::Vector3d di,dj,vi,vj;
  double deltaTime;
  double info,dis;
};
struct UWBFactor_connect_2time_plus_mul
{
  UWBFactor_connect_2time_plus_mul(Eigen::Vector3d pi,Eigen::Quaterniond qi,
  Eigen::Vector3d pj,Eigen::Quaterniond qj,
  double para_hinge[],double _dis1,double _dis2,double _info)
  {
    di=pi,dj=pj;
    ri=qi,rj=qj;
    di+=ri.toRotationMatrix()*(Eigen::Vector3d(para_hinge));
    dj+=rj.toRotationMatrix()*(Eigen::Vector3d(para_hinge));
    info=_info;
    dis1=_dis1;
    dis2=_dis2;
  }
  template <typename T>
  bool operator()(const T*  pi,const T* yi,const T* pj,const T*yj,const T* an,const T*bias,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Vi,Di,Pj,Dj,An;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Pj(i)=pi[i];
        Di(i)=(T)di(i);
        Dj(i)=(T)dj(i);
        An(i)=an[i];
    }
    Eigen::Matrix<T, 3, 3>Yawi=fromYawToMat(yi[0]),Yawj=fromYawToMat(yj[0]);
    T len1=(T)dis1;
    T len2=(T)dis2;
    //len1/len2=beta*(pi-an)+gama/beta*(pj-an)+gama
    //len1-len2=beta*(||pi-an||-||pj-an||)
    Pj=Yawj*Dj+Pj;
    Pi=Yawi*Di+Pi;
    Eigen::Matrix<T,3,1>bet1=Pi-An,bet2=Pj-An;
    T est_len_1=(bet1.norm()*bias[1]+bias[0])*len2;
    T est_len_2=(bet2.norm()*bias[1]+bias[0])*len1;
    residuals[0]=(est_len_1-est_len_2)/(T(info));
    //residuals[1]=(len1-len2-(bias[1]*(bet1.norm()-bet2.norm())))/(T(info));
    return true;
  }
  Eigen::Quaterniond ri,rj;
  Eigen::Vector3d di,dj,vi,vj;
  double deltaTime;
  double info,dis1,dis2;
};
struct UWBFactor_connect_pos
{
  UWBFactor_connect_pos(Eigen::Vector3d pi,double _dis,double _info)
  {
    di=pi;
    info=_info;
    dis=_dis;
  }
  template <typename T>
  bool operator()(const T* pi,const T*bias,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Di;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Di(i)=(T)di(i);
    }
    T len=(T)dis-bias[0];
    Eigen::Matrix<T,3,1>bet=Pi-Di;
    T est_len=bet.norm();
    residuals[0]=(est_len-len)/(T(info));
    return true;
  }
  Eigen::Quaterniond ri,rj;
  Eigen::Vector3d di,dj,vi,vj;
  double deltaTime;
  double info,dis;
};
struct UWBFactor_anchor_and_anchor
{
  UWBFactor_anchor_and_anchor(double _dis,double _info)
  {
    info=_info;
    dis=_dis;
  }
  template <typename T>
  bool operator()(const T*  pi,const T* pj,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Pj;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Pj(i)=pj[i];
    }
    T len=(T)dis;
    Eigen::Matrix<T,3,1>bet=Pi-Pj;
    T est_len=bet.norm();
    residuals[0]=(est_len-len)/(T(info));
    return true;
  }
  double deltaTime;
  double info,dis;
};




struct UWBFactor_kin_len
{
  UWBFactor_kin_len(Eigen::Vector3d pi,Eigen::Quaterniond qi,
                    Eigen::Vector3d pj,Eigen::Quaterniond qj,
                    double para_hinge[],double _dis,double _info)
  {
    di=pi;
    dj=pj;
    ri=qi;
    rj=qj;
    di+=ri.toRotationMatrix()*(Eigen::Vector3d(para_hinge));
    dj+=rj.toRotationMatrix()*(Eigen::Vector3d(para_hinge));
    info=_info;
    dis=_dis;
  }
  template <typename T>
  bool operator()(const T*  pi,const T* yi,const T* pj,const T* yj,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Vi,Di,Pj,Dj;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Di(i)=(T)di(i);
        Pj(i)=pj[i];
        Dj(i)=(T)dj(i);
    }
    Eigen::Matrix<T, 3, 3>Yawi=fromYawToMat(yi[0]),Yawj=fromYawToMat(yj[0]);
    Pi=Yawi*Di+Pi;
    Pj=Yawj*Dj+Pj;
    T len=(Pi-Pj).norm();
    residuals[0]=((T)dis-len)/(T(info));
    return true;
  }
  Eigen::Quaterniond ri,rj;
  Eigen::Vector3d di,dj,vi,vj;
  double deltaTime;
  double info,dis;
};

struct UWBFactor_kin_att
{
  UWBFactor_kin_att(Eigen::Vector3d pi,Eigen::Quaterniond qi,
                    Eigen::Vector3d pj,Eigen::Quaterniond qj,
                    Eigen::Vector3d pk,Eigen::Quaterniond qk,
                    double para_hinge[],double _z_val,double _info)
  {
    di=pi;
    dj=pj;
    ri=qi;
    rj=qj;
    dk=pk;
    rk=qk;
    di+=ri.toRotationMatrix()*(Eigen::Vector3d(para_hinge));
    dj+=rj.toRotationMatrix()*(Eigen::Vector3d(para_hinge));
    dk+=rk.toRotationMatrix()*(Eigen::Vector3d(para_hinge));
    info=_info;
    z_val=_z_val;
  }
  template <typename T>
  bool operator()(const T*  pi,const T* yi,const T* pj,const T* yj,const T* pk,const T* yk,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Vi,Di,Pj,Dj,Pk,Dk;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Di(i)=(T)di(i);
        Pj(i)=pj[i];
        Dj(i)=(T)dj(i);
        Pk(i)=pk[i];
        Dk(i)=(T)dk[i];
    }
    Eigen::Matrix<T, 3, 3>Yawi=fromYawToMat(yi[0]),Yawj=fromYawToMat(yj[0]),
    Yawk=fromYawToMat(yk[0]);
    Pi=Yawi*Di+Pi;
    Pj=Yawj*Dj+Pj;
    Pk=Yawk*Dk+Pk;
    Eigen::Matrix<T, 3, 1> Line1=Pi-Pj,Line2=Pi-Pk;
    Eigen::Matrix<T, 3, 1> N = Line1.cross(Line2);
    N.normalize();
    residuals[0] = (N(2) - (T)z_val) / (T)info;
    return true;
  }
  Eigen::Quaterniond ri,rj,rk;
  Eigen::Vector3d di,dj,dk,vi,vj;
  double deltaTime;
  double info,z_val;
};




