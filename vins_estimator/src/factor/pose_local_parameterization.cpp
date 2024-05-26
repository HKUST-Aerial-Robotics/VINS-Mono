#include "pose_local_parameterization.h"

bool PoseManifold::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQuat(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}

bool PoseManifold::Minus(const double *y, const double *x, double *y_minues_x) const
{
    throw std::runtime_error("Minus function not implemented.");
    // Eigen::Map<const Eigen::Vector3d> _p_x(x);
    // Eigen::Map<const Eigen::Quaterniond> _q_x(x + 3);
    // Eigen::Map<const Eigen::Vector3d> _p_y(y);
    // Eigen::Map<const Eigen::Quaterniond> _q_y(y + 3);

    // Eigen::Map<Eigen::Vector3d> dp(y_minues_x);
    // Eigen::Map<Eigen::Quaterniond> dq(y_minues_x + 3);
    // // TODO: Understand the need for delta q. Might to half some values

    // dp = _p_y - _p_x;
    // dq = _q_y.conjugate() * _q_y;

    return false;
}
bool PoseManifold::PlusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}
bool PoseManifold::MinusJacobian(const double *x, double *jacobian) const
{
    throw std::runtime_error("MinusJacobian function not implemented.");
    // Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    // j.topRows<6>().setIdentity();
    // j.bottomRows<1>().setZero();

    return false;
}