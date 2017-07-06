#include "perspective_factor.h"
Eigen::Matrix2d PerspectiveFactor::sqrt_info;
double PerspectiveFactor::sum_t;

PerspectiveFactor::PerspectiveFactor(const Eigen::Vector2d &_pts_2d, const Eigen::Vector3d &_pts_3d, const int _track_num)
:pts_2d(_pts_2d), pts_3d(_pts_3d), track_num(_track_num){};

bool PerspectiveFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    
    Eigen::Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    
    Eigen::Vector3d pts_imu_i = Qi.inverse() * (pts_3d - Pi);
    Eigen::Vector3d pts_camera_i = qic.inverse() * (pts_imu_i - tic);
    
    double dep_i = pts_camera_i.z();
    
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual = (pts_camera_i / dep_i).head<2>() - pts_2d.head<2>();
    residual = sqrt_info * residual * (double)(track_num) / 10.0;
    
    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        reduce << 1. / dep_i, 0, -pts_camera_i(0) / (dep_i * dep_i),
        0, 1. / dep_i, -pts_camera_i(1) / (dep_i * dep_i);
        
        reduce = sqrt_info * reduce * (double)(track_num) / 10.0;
        
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            
            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = -ric.transpose() * Ri.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_i);
            
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }
        
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[1]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = -ric.transpose();
            jaco_ex.rightCols<3>() = Utility::skewSymmetric(pts_camera_i);
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
    }
    
    return true;
}