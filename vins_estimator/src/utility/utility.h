/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
typedef Eigen::Matrix<double,6,1> Vector6d;
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


class Utility
{
  public:
    template <typename T>
    static Eigen::Matrix<T,3,1> R2ypr2(const Eigen::Matrix<T,3,3> &R)
    {
        Eigen::Matrix<T,3,1> n = R.col(0);
        Eigen::Matrix<T,3,1> o = R.col(1);
        Eigen::Matrix<T,3,1> a = R.col(2);

        Eigen::Matrix<T,3,1> ypr(3);
        T y = atan2(n(1), n(0));
        T p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        T r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / (T)M_PI * (T)180.0;
    }
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        return ans;
    }

    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

    static Eigen::Matrix3d g2R(const Eigen::Vector3d &g);

    template <size_t N>
    struct uint_
    {
    };

    template <size_t N, typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<N>)
    {
        unroller(f, iter, uint_<N - 1>());
        f(iter + N);
    }

    template <typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<0>)
    {
        f(iter);
    }

    template <typename T>
    static T normalizeAngle(const T& angle_degrees) {
      T two_pi(2.0 * 180);
      if (angle_degrees > 0)
      return angle_degrees -
          two_pi * std::floor((angle_degrees + T(180)) / two_pi);
      else
        return angle_degrees +
            two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
    };
    template <typename T>
    static T normalizeAngleByAng(const T& angle_degrees) {
      if (angle_degrees > T(180.0))
            return angle_degrees - T(360.0);
      else if (angle_degrees < T(-180.0))
            return angle_degrees + T(360.0);
      else
            return angle_degrees;
    };
    /*
        三点确定一个平面 a(x-x0)+b(y-y0)+c(z-z0)=0  --> ax + by + cz + d = 0   d = -(ax0 + by0 + cz0)
        平面通过点（x0,y0,z0）以及垂直于平面的法线（a,b,c）来得到
        (a,b,c)^T = vector(AO) cross vector(BO)
        d = O.dot(cross(AO,BO))
    */
    static Eigen::Vector4d pi_from_ppp(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3) {
        Eigen::Vector4d pi;
        pi << ( x1 - x3 ).cross( x2 - x3 ), - x3.dot( x1.cross( x2 ) );
        return pi;
    }
    // 两平面相交得到直线的plucker 坐标
    static Vector6d pipi_plk( Eigen:: Vector4d pi1, Eigen:: Vector4d pi2){
        Vector6d plk;
        Eigen::Matrix4d dp = pi1 * pi2.transpose() - pi2 * pi1.transpose();

        plk << dp(0,3), dp(1,3), dp(2,3), - dp(1,2), dp(0,2), - dp(0,1);
        return plk;
    }
    static Eigen::Vector4d plk_to_orth(Vector6d plk)
    {
        Eigen::Vector4d orth;
        Eigen::Vector3d n = plk.head(3);
        Eigen::Vector3d v = plk.tail(3);

        Eigen::Vector3d u1 = n/n.norm();
        Eigen::Vector3d u2 = v/v.norm();
        Eigen::Vector3d u3 = u1.cross(u2);

        // todo:: use SO3
        orth[0] = atan2( u2(2),u3(2) );
        orth[1] = asin( -u1(2) );
        orth[2] = atan2( u1(1),u1(0) );

        Eigen::Vector2d w( n.norm(), v.norm() );
        w = w/w.norm();
        orth[3] = asin( w(1) );

        return orth;

    }
    
    static Vector6d plk_to_pose( Vector6d plk_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw ) {
        Eigen::Vector3d nw = plk_w.head(3);
        Eigen::Vector3d vw = plk_w.tail(3);

        Eigen::Vector3d nc = Rcw * nw + skewSymmetric(tcw) * Rcw * vw;
        Eigen::Vector3d vc = Rcw * vw;

        Vector6d plk_c;
        plk_c.head(3) = nc;
        plk_c.tail(3) = vc;
        return plk_c;
    }
    // 世界坐标系到相机坐标系下
    static Vector6d plk_from_pose( Vector6d plk_c, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw ) {

        Eigen::Matrix3d Rwc = Rcw.transpose();
        Eigen::Vector3d twc = -Rwc*tcw;
        return plk_to_pose( plk_c, Rwc, twc);
    }


    static Vector6d orth_to_plk(Eigen::Vector4d orth)
    {
        Vector6d plk;

        Eigen::Vector3d theta = orth.head(3);
        double phi = orth[3];

        Eigen::Matrix3d R=ypr2R(theta);
        double s1 = sin(theta[0]);
        double c1 = cos(theta[0]);
        double s2 = sin(theta[1]);
        double c2 = cos(theta[1]);
        double s3 = sin(theta[2]);
        double c3 = cos(theta[2]);

        Eigen::Matrix3d R2;
        R2 <<
        c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
                c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
                -s2,                  s1 * c2,                  c1 * c2;
        Eigen::Matrix3d R3=R2-R;
        double diff=R3.lpNorm<1>();
        //printf("ypr2R and hand calc diff ==%lf",diff);
        double w1 = cos(phi);
        double w2 = sin(phi);
        double d = w1/w2;      // 原点到直线的距离
        Eigen::Vector3d u1 = R.col(0);
        Eigen::Vector3d u2 = R.col(1);
        Eigen::Vector3d n = w1 * u1;
        Eigen::Vector3d v = w2 * u2;
        plk.head(3) = n;
        plk.tail(3) = v;
        return plk;
    }
    static double gauss(double sigma, double x) {
        double expVal = -1 * (pow(x, 2) / pow(2 * sigma, 2));
        double divider = sqrt(2 * M_PI * pow(sigma, 2));
        return (1 / divider) * exp(expVal);
    }

    template <typename T>
    static Eigen::Matrix<T,3,3> fromYawToMat(T yaw)
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
};
