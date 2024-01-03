#include "line_projection_factor.h"
#include "line_parameterization.h"


Eigen::Matrix2d lineProjectionFactor::sqrt_info;
double lineProjectionFactor::sum_t;

lineProjectionFactor::lineProjectionFactor(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i){};

/*
  parameters[0]:  Twi
  parameters[1]:  Tbc
  parameters[2]:  line_orth
*/
bool lineProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector4d line_orth( parameters[2][0],parameters[2][1],parameters[2][2],parameters[2][3]);
    Vector6d line_w = Utility::orth_to_plk(line_orth);
    Eigen::Matrix3d Rwb(Qi);
    Eigen::Vector3d twb(Pi);
    Vector6d line_b =Utility:: plk_from_pose(line_w, Rwb, twb);


    Eigen::Matrix3d Rbc(qic);
    Eigen::Vector3d tbc(tic);
    Vector6d line_c =Utility:: plk_from_pose(line_b, Rbc, tbc);


    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_c.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    residual = sqrt_info * residual;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;

        if (jacobians[0])
        {
            //std::cout <<"jacobian_pose_i"<<"\n";
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 6, 6> invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*Utility::skewSymmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Eigen::Vector3d nw = line_w.head(3);
            Eigen::Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwb.transpose() * Utility::skewSymmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = Utility::skewSymmetric( Rwb.transpose() * (nw + Utility::skewSymmetric(dw) * twb) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = Utility::skewSymmetric( Rwb.transpose() * dw);

            jaco_Lc_pose = invTbc * jaco_Lc_pose;
            //std::cout <<invTbc<<"\n"<<jaco_Lc_pose<<"\n\n";

            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            //std::cout <<jacobian_pose_i<<"\n\n";

            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }
        if (jacobians[1])
        {

            //std::cout <<"jacobian_ex_pose"<<"\n";
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[1]);

            Eigen::Vector3d nb = line_b.head(3);
            Eigen::Vector3d db = line_b.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_ex;
            jaco_Lc_ex.setZero();
            jaco_Lc_ex.block(0,0,3,3) = Rbc.transpose() * Utility::skewSymmetric(db);   // Lc_t
            jaco_Lc_ex.block(0,3,3,3) = ( Rbc.transpose() * (nb + Utility::skewSymmetric(db) * tbc) );  // Lc_theta
            jaco_Lc_ex.block(3,3,3,3) = Utility::skewSymmetric( Rbc.transpose() * db);

            jacobian_ex_pose.leftCols<6>() = jaco_e_Lc * jaco_Lc_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[2]);

            Eigen::Matrix3d Rwc = Rwb * Rbc;
            Eigen::Vector3d twc = Rwb * tbc + twb;
            Eigen::Matrix<double, 6, 6> invTwc;
            invTwc << Rwc.transpose(), -Rwc.transpose() * Utility::skewSymmetric(twc),
                    Eigen::Matrix3d::Zero(),  Rwc.transpose();
            //std::cout<<invTwc<<"\n";

            Eigen::Vector3d nw = line_w.head(3);
            Eigen::Vector3d vw = line_w.tail(3);
            Eigen::Vector3d u1 = nw/nw.norm();
            Eigen::Vector3d u2 = vw/vw.norm();
            Eigen::Vector3d u3 = u1.cross(u2);
            Eigen::Vector2d w( nw.norm(), vw.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lw_orth;
            jaco_Lw_orth.setZero();
            jaco_Lw_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lw_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lw_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lw_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(3,3,3,1) = w(0) * u2;

            //std::cout<<jaco_Lw_orth<<"\n";

            jacobian_lineOrth = jaco_e_Lc * invTwc * jaco_Lw_orth;
        }
    }
    return true;
}

Eigen::Matrix2d lineProjectionFactor_incamera::sqrt_info;
lineProjectionFactor_incamera::lineProjectionFactor_incamera(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};
/*
  parameters[0]:  Twi
  parameters[1]:  Twj
  parameters[2]:  Tbc
  parameters[3]:  line_orth
*/
bool lineProjectionFactor_incamera::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector4d line_orth( parameters[3][0],parameters[3][1],parameters[3][2],parameters[3][3]);
    Vector6d line_ci = Utility::orth_to_plk(line_orth);

    Eigen::Matrix3d Rbc(qic);
    Eigen::Vector3d tbc(tic);
    Vector6d line_bi = Utility::plk_to_pose(line_ci, Rbc, tbc);

    Eigen::Matrix3d Rwbi = Qi.toRotationMatrix();
    Eigen::Vector3d twbi(Pi);
    Vector6d line_w = Utility::plk_to_pose(line_bi, Rwbi, twbi);

    Eigen::Matrix3d Rwbj = Qj.toRotationMatrix();
    Eigen::Vector3d twbj(Pj);
    Vector6d line_bj = Utility::plk_from_pose(line_w, Rwbj, twbj);

    Vector6d line_cj = Utility::plk_from_pose(line_bj, Rbc, tbc);

    Eigen::Vector3d nc = line_cj.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;
    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            Eigen::Matrix3d Rwcj = Rwbj * Rbc;
            Eigen::Vector3d twcj = Rwbj * tbc + twbj;
            Eigen::Matrix<double, 6, 6> invTwcj;
            invTwcj << Rwcj.transpose(), -Rwcj.transpose()*Utility::skewSymmetric(twcj),
                    Eigen::Matrix3d::Zero(),  Rwcj.transpose();

            Eigen::Vector3d nbi = line_bi.head(3);
            Eigen::Vector3d dbi = line_bi.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = - Utility::skewSymmetric(Rwbi * dbi);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = -Rwbi * Utility::skewSymmetric( nbi) - Utility::skewSymmetric(twbi) * Rwbi * Utility::skewSymmetric(dbi);  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = -Rwbi * Utility::skewSymmetric(dbi);

            jaco_Lc_pose = invTwcj * jaco_Lc_pose;
            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;
            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 6, 6> invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*Utility::skewSymmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Eigen::Vector3d nw = line_w.head(3);
            Eigen::Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwbj.transpose() * Utility::skewSymmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = Utility::skewSymmetric( Rwbj.transpose() * (nw + Utility::skewSymmetric(dw) * twbj) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = Utility::skewSymmetric( Rwbj.transpose() * dw);

            jaco_Lc_pose = invTbc * jaco_Lc_pose;
            jacobian_pose_j.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            jacobian_pose_j.rightCols<1>().setZero();            //最后一列设成0
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);

            Eigen::Matrix3d Rbjbi = Rwbj.transpose() * Rwbi;
            Eigen::Matrix3d Rcjci = Rbc.transpose() * Rbjbi * Rbc;
            Eigen::Vector3d tcjci = Rbc * ( Rwbj.transpose() * (Rwbi * tbc + twbi - twbj) - tbc);

            Eigen::Vector3d nci = line_ci.head(3);
            Eigen::Vector3d dci = line_ci.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_ex;
            jaco_Lc_ex.setZero();
            jaco_Lc_ex.block(0,0,3,3) = -Rbc.transpose() * Rbjbi * Utility::skewSymmetric( Rbc * dci) + Rbc.transpose() * Utility::skewSymmetric(Rbjbi * Rbc * dci);   // Lc_t
            Eigen::Matrix3d tmp = Utility::skewSymmetric(tcjci) * Rcjci;
            jaco_Lc_ex.block(0,3,3,3) = -Rcjci * Utility::skewSymmetric(nci) + Utility::skewSymmetric(Rcjci * nci)
                                        -tmp * Utility::skewSymmetric(dci) + Utility::skewSymmetric(tmp * dci);    // Lc_theta
            jaco_Lc_ex.block(3,3,3,3) = -Rcjci * Utility::skewSymmetric(dci) + Utility::skewSymmetric(Rcjci * dci);

            jacobian_ex_pose.leftCols<6>() = jaco_e_Lc * jaco_Lc_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[3]);

            Eigen::Matrix3d Rbjbi = Rwbj.transpose() * Rwbi;
            Eigen::Matrix3d Rcjci = Rbc.transpose() * Rbjbi * Rbc;
            Eigen::Vector3d tcjci = Rbc * ( Rwbj.transpose() * (Rwbi * tbc + twbi - twbj) - tbc);

            Eigen::Matrix<double, 6, 6> Tcjci;
            Tcjci << Rcjci, Utility::skewSymmetric(tcjci) * Rcjci,
                    Eigen::Matrix3d::Zero(),  Rcjci;

            Eigen::Vector3d nci = line_ci.head(3);
            Eigen::Vector3d vci = line_ci.tail(3);
            Eigen::Vector3d u1 = nci/nci.norm();
            Eigen::Vector3d u2 = vci/vci.norm();
            Eigen::Vector3d u3 = u1.cross(u2);
            Eigen::Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lc_orth;
            jaco_Lc_orth.setZero();
            jaco_Lc_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lc_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lc_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lc_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lc_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lc_orth.block(3,3,3,1) = w(0) * u2;

            jacobian_lineOrth = jaco_e_Lc * Tcjci * jaco_Lc_orth;
        }
    }
    return true;
}

Eigen::Matrix2d lineProjectionFactor_instartframe::sqrt_info;
lineProjectionFactor_instartframe::lineProjectionFactor_instartframe(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};
/*
  parameters[0]:  Twi
  parameters[1]:  Twj
  parameters[2]:  Tbc
  parameters[3]:  line_orth
*/
bool lineProjectionFactor_instartframe::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector4d line_orth( parameters[0][0],parameters[0][1],parameters[0][2],parameters[0][3]);
    Vector6d line_ci = Utility::orth_to_plk(line_orth);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_ci.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;
    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    //std::cout<< residual <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[0]);
            Eigen::Vector3d nci = line_ci.head(3);
            Eigen::Vector3d vci = line_ci.tail(3);
            Eigen::Vector3d u1 = nci/nci.norm();
            Eigen::Vector3d u2 = vci/vci.norm();
            Eigen::Vector3d u3 = u1.cross(u2);
            Eigen::Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();
            Eigen::Matrix<double, 6, 4> jaco_Lci_orth;
            jaco_Lci_orth.setZero();
            jaco_Lci_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lci_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lci_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lci_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lci_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lci_orth.block(3,3,3,1) = w(0) * u2;

            jacobian_lineOrth = jaco_e_Lc  * jaco_Lci_orth;
        }

    }
    return true;
}