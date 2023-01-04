//
// Created by lacie on 10/10/2022.
//

#ifndef SEMANTICSLAM_PROJECTIONONEFRAMETWOCAMFACTOR_H
#define SEMANTICSLAM_PROJECTIONONEFRAMETWOCAMFACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility.h"
#include "tic_toc.h"
#include "parameters.h"

class ProjectionOneFrameTwoCamFactor : public ceres::SizedCostFunction<2, 7, 7, 1, 1>
{
public:
    ProjectionOneFrameTwoCamFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                                   const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
                                   const double _td_i, const double _td_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d velocity_i, velocity_j;
    double td_i, td_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

#endif //SEMANTICSLAM_PROJECTIONONEFRAMETWOCAMFACTOR_H
