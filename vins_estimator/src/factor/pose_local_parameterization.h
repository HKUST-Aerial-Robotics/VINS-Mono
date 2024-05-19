#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

class PoseManifold : public ceres::Manifold
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool Minus(const double *y, const double *x, double *y_minus_x) const;
    virtual bool PlusJacobian(const double *x, double *jacobian) const;
    virtual bool MinusJacobian(const double *x, double *jacobian) const;
    virtual int AmbientSize() const { return 7; };
    virtual int TangentSize() const { return 6; };
};
