#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>

// Zero-Velocity Update (ZUPT) factors for the hover-aware fork.
//
// Injected into the Ceres problem when the motion detector confirms the
// platform is stationary (velocity-and-position pin), or rotating in place
// (position-only pin). They stop IMU-bias-driven drift during prolonged
// hover, the regime where visual parallax is degenerate.
//
// The sqrt-information scaling is adaptive — computed in estimator.cpp from
// the IMU noise model (ACC_N) and the observed frame period so the weights
// reflect the *actual* pseudo-measurement noise, not a hardcoded constant.
// Per-axis weights are supported so Z (gravity-aligned in world frame) can
// be decoupled from XY when needed.

class ZUPTVelocityFactor : public ceres::SizedCostFunction<3, 9>
{
  public:
    explicit ZUPTVelocityFactor(double weight) { setWeight(weight); }
    explicit ZUPTVelocityFactor(const Eigen::Vector3d &weight_xyz) { setWeight(weight_xyz); }

    void setWeight(double weight)
    { sqrt_info_ = weight * Eigen::Matrix3d::Identity(); }

    void setWeight(const Eigen::Vector3d &weight_xyz)
    {
        sqrt_info_.setZero();
        sqrt_info_(0, 0) = weight_xyz(0);
        sqrt_info_(1, 1) = weight_xyz(1);
        sqrt_info_(2, 2) = weight_xyz(2);
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {
        Eigen::Vector3d Vj(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual = sqrt_info_ * Vj;

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();
            J.block<3, 3>(0, 0) = sqrt_info_;
        }
        return true;
    }

  private:
    Eigen::Matrix3d sqrt_info_;
};

class ZUPTPositionFactor : public ceres::SizedCostFunction<3, 7, 7>
{
  public:
    explicit ZUPTPositionFactor(double weight) { setWeight(weight); }
    explicit ZUPTPositionFactor(const Eigen::Vector3d &weight_xyz) { setWeight(weight_xyz); }

    void setWeight(double weight)
    { sqrt_info_ = weight * Eigen::Matrix3d::Identity(); }

    void setWeight(const Eigen::Vector3d &weight_xyz)
    {
        sqrt_info_.setZero();
        sqrt_info_(0, 0) = weight_xyz(0);
        sqrt_info_(1, 1) = weight_xyz(1);
        sqrt_info_(2, 2) = weight_xyz(2);
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const
    {
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual = sqrt_info_ * (Pj - Pi);

        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> Ji(jacobians[0]);
                Ji.setZero();
                Ji.block<3, 3>(0, 0) = -sqrt_info_;
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> Jj(jacobians[1]);
                Jj.setZero();
                Jj.block<3, 3>(0, 0) = sqrt_info_;
            }
        }
        return true;
    }

  private:
    Eigen::Matrix3d sqrt_info_;
};
