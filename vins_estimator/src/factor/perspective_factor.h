#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class PerspectiveFactor : public ceres::SizedCostFunction<2, 7, 7>
{
public:
    PerspectiveFactor(const Eigen::Vector2d &_pts_2d, const Eigen::Vector3d &_pts_3d, const int _track_num);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    
    Eigen::Vector3d pts_3d;
    Eigen::Vector2d pts_2d;
    int track_num;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};