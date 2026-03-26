#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

namespace opt {

using VecX  = Eigen::VectorXd;
using MatXX = Eigen::MatrixXd;
using Vec3  = Eigen::Vector3d;

inline double normalize_angle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

} // namespace opt
