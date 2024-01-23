#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"

template <typename T>
Eigen::Matrix<T,3,3> skewSymmetricTemplate(Eigen::Matrix<T,3,1> w)
{
  Eigen::Matrix<T,3,3> ans;
  ans << T(0), -w(2), w(1),
            w(2), T(0), -w(0),
            -w(1), w(0), T(0);
  return ans;
}

struct kinFactor_bet_4dof_1
{
  kinFactor_bet_4dof_1(double POSEI[],double YawI[], double POSEJ[],double YawJ[],double _deltaTime, Eigen::Matrix<double,4,1> _info)
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


struct kinFactor_old
{
  kinFactor_old(double POSEI[], double _info1,double _info2)
  {
    di=Eigen::Vector3d(POSEI[0],POSEI[1],POSEI[2]);
    qi=Eigen::Quaterniond(POSEI[6],POSEI[3],POSEI[4],POSEI[5]);
    info1=_info1;
    info2=_info2;
  }
  template <typename T>
  bool operator()(const T*  pi,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi,Pj,E;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
        Pj(i)=(T)di[i];
    }
    E=Pi-Pj;
    Eigen::Quaternion<T>Qi(pi[6],pi[3],pi[4],pi[5]);
    Eigen::Quaternion<T>Qj((T)qi.w(),(T)qi.x(),(T)qi.y(),(T)qi.z());
    Eigen::Quaternion<T>Qk=Qj*Qi.inverse();
    Qk.normalize();
    for(int i=0;i<=2;i++){
      residuals[i]=E(i)/(T)info1;
    }
    residuals[3]=Qk.x()/(T)info2;
    residuals[4]=Qk.y()/(T)info2;
    residuals[5]=Qk.z()/(T)info2;
    residuals[6]=(Qk.w()-(T)1.00)/(T)info2;
    return true;
  }
  Eigen::Vector3d di;
  Eigen::Quaterniond qi;
  double info1,info2;
};

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
struct kinFactor_bet_old_4dof_1
{
  kinFactor_bet_old_4dof_1(double POSEI[],Eigen::Matrix<double,7,1>_info)
  {
    di=Eigen::Vector3d(POSEI[0],POSEI[1],POSEI[2]);
    yawi=POSEI[3];
    info=_info;
  }
  template <typename T>
  bool operator()(const T*  pi,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
    }

    Eigen::Matrix<T, 3, 1> pre_p((T)di.x(),(T)di.y(),(T)di.z());//= dj-di;
    residuals[0]=pre_p[0]-Pi[0];
    residuals[1]=pre_p[1]-Pi[1];
    residuals[2]=pre_p[2]-Pi[2];
    residuals[3]=yawi-pi[3];
    for(int i=0;i<=3;i++){
      residuals[i]=residuals[i]*(T)(1/info(i));
    }
    return true;
  }
  void print()
  {
    ROS_INFO("print() %d %lf %lf %lf %lf %lf %lf",AGENT_NUMBER,dj.x(),dj.y(),dj.z(),vj.x(),vj.y(),vj.z());
  }
  Eigen::Vector3d di,dj,vi,vj,delta_d,delta_v;
  double yawi;
  double pixi;
  double pixj;
  double deltaTime;
  Eigen::Matrix<double,7,1>info;
};
struct kinFactor_bet_old_4dof_2
{
  kinFactor_bet_old_4dof_2(double POSEI[],double YawI[],Eigen::Matrix<double,4,1>_info)
  {
    di=Eigen::Vector3d(POSEI[0],POSEI[1],POSEI[2]);
    yawi=YawI[0];
    info=_info;
  }
  template <typename T>
  bool operator()(const T*  pi,const T*  yi,T* residuals) const
  {
    Eigen::Matrix<T, 3, 1> Pi;
    for(int i=0;i<3;i++)
    {
        Pi(i)=pi[i];
    }

    Eigen::Matrix<T, 3, 1> pre_p((T)di.x(),(T)di.y(),(T)di.z());//= dj-di;
    residuals[0]=pre_p[0]-Pi[0];
    residuals[1]=pre_p[1]-Pi[1];
    residuals[2]=pre_p[2]-Pi[2];
    residuals[3]=yawi-yi[0];
    for(int i=0;i<=3;i++){
      residuals[i]=residuals[i]*(T)(1/info(i));
    }
    return true;
  }
  void print()
  {
    ROS_INFO("print() %d %lf %lf %lf %lf %lf %lf",AGENT_NUMBER,dj.x(),dj.y(),dj.z(),vj.x(),vj.y(),vj.z());
  }
  Eigen::Vector3d di,dj,vi,vj,delta_d,delta_v;
  double yawi;
  double pixi;
  double pixj;
  double deltaTime;
  Eigen::Matrix<double,4,1>info;
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
struct kinFactor_connect_4dof_2
{
  kinFactor_connect_4dof_2(Eigen::Vector3d pi,Eigen::Quaterniond qi,Eigen::Vector3d pj,Eigen::Quaterniond qj,double para_hinge,double _info)
  {
    di=pi;
    dj=pj;
    ri=qi;
    rj=qj;
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
struct kinFactor_connect_4dof_tight
{
  kinFactor_connect_4dof_tight(Eigen::Vector3d _wp_1, Eigen::Matrix3d _wr_1,Eigen::Vector3d _wp_2,Eigen::Matrix3d _wr_2,double _info)
  {
    wp[0]=_wp_1;
    wr[0]=_wr_1;
    wp[1]=_wp_2;
    wr[1]=_wr_2;
    info=_info;
  }
  template <typename T>
  bool operator()(const T*  pi,const T* pj,const T* hinge,const T* length,T* residuals) const
  {
    
    Eigen::Matrix<T, 3, 1> Pi[2],WP[2],OT,dis;
    for(int i=0;i<3;i++)
    {
        Pi[0](i)=pi[i];
        Pi[1](i)=pj[i];
        WP[0](i)=(T)wp[0](i);
        WP[1](i)=(T)wp[1](i);
    }
    Eigen::Matrix<T, 3, 3>Yawi=fromYawToMat(pj[3]);
    Eigen::Quaternion<T> Qi[2]{Eigen::Quaternion<T>{pi[6],pi[3],pi[4],pi[5]}};
    Eigen::Matrix<T,3,3> WR[2];
    for(int i=0;i<=2;i++)for(int j=0;j<=2;j++)WR[0](i,j)=(T)wr[0](i,j),WR[1](i,j)=(T)wr[1](i,j);

    Pi[1]=WP[1];

    Pi[0]+=Qi[0].toRotationMatrix()*Eigen::Matrix<T, 3, 1>(hinge);
    Pi[0]=WR[0]*Pi[0]+WP[0];
    dis=Pi[1]-Pi[0];
    
    T est_len=dis.norm();
    T len=length[0];
    residuals[0]=(est_len-len)/(T(info));
    return true;
  }
  Eigen::Vector3d wp[2];
  Eigen::Matrix3d wr[2];
  Eigen::Vector3d ot;
  double pixi;
  double pixj;
  int Two;
  double deltaTime;
  double info;
};
struct kinFactor_connect_hyp_4dof_tight
{
  kinFactor_connect_hyp_4dof_tight(Eigen::Vector3d _wp1,Eigen::Matrix3d _wr1,Eigen::Vector3d _wp2,
  Eigen::Matrix3d _wr2,Eigen::Vector3d _wp3,Eigen::Matrix3d _wr3,double _z_val,double _info)
  {
    wp[0]=_wp1;
    wr[0]=_wr1;
    wp[1]=_wp2;
    wr[1]=_wr2;
    wp[2]=_wp3;
    wr[2]=_wr3;
    info=_info;
    z_val=_z_val;
  }
  template <typename T>
  bool operator()(const T*  pi,const T*  pj,const T*  pk,const T* hinge,T* residuals) const
  {
    
    Eigen::Matrix<T, 3, 1> Pi[3],WP[3],OT,dis,OT2;
    Eigen::Matrix<T, 4, 1> DT[3];
    for(int i=0;i<3;i++)
    {
        Pi[0](i)=pi[i];
        Pi[1](i)=pj[i];
        Pi[2](i)=pk[i];
        for(int k=0;k<=2;k++)WP[k](i)=(T)wp[k](i);
    }
    for(int i=0;i<=3;i++){
      DT[0](i)=pi[i+3];
    }
    Eigen::Quaternion<T> Qi[3];
    for(int i=0;i<=0;i++)Qi[i]=Eigen::Quaternion<T>{DT[i](3),DT[i](0),DT[i](1),DT[i](2)};
    Qi[1]=Eigen::Quaternion<T>{fromYawToMat(pj[3])};
    Qi[2]=Eigen::Quaternion<T>{fromYawToMat(pk[3])};
    Eigen::Matrix<T,3,3> WR[3];
    for(int k=0;k<=2;k++){
      for(int i=0;i<=2;i++){
        for(int j=0;j<=2;j++){
          WR[k](i,j)=(T)wr[k](i,j);
        }
      }
      
    }
    Pi[0]+=Qi[0].toRotationMatrix()*Eigen::Matrix<T, 3, 1>(hinge);
    Pi[0]=WR[0]*Pi[0]+WP[0];
    
    Pi[1]=WP[1];
    Pi[2]=WP[2];
    //cout<<Pi[1](0)<<"  "<<Pi[1](1)<<" "<<Pi[1](2)<<Pi[2](0)<<"  "<<Pi[2](1)<<" "<<Pi[2](2) <<endl;
    Eigen::Matrix<T,3,1>Line1=Pi[0]-Pi[1];
    Eigen::Matrix<T,3,1>Line2=Pi[0]-Pi[2];
    Eigen::Matrix<T,3,1>N=Line1.cross(Line2);
    N.normalize();
    residuals[0]=(N(2)-(T)z_val)/info;
    return true;
  }
  Eigen::Vector3d wp[3];
  Eigen::Matrix3d wr[3];
  int Two;
  double info;
  double z_val;
};
struct kinFactor_connect_hyp_4dof_2
{
  kinFactor_connect_hyp_4dof_2(Eigen::Vector3d pi,Eigen::Quaterniond qi,Eigen::Vector3d pj,Eigen::Quaterniond qj,
  Eigen::Vector3d pk,Eigen::Quaterniond qk,double para_hinge,double _val,double _info)
  {
    di=pi;
    dj=pj;
    dk=pk;
    ri=qi;
    rj=qj;
    rk=qk;
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


