#pragma once

#include "edge.hpp"
#include "vertex.hpp"
#include <cmath>

namespace opt {

/// @brief Relative pose constraint between two PoseVertex nodes
/// @details Measurement z = (dx, dy, dtheta) in the local frame of vertex i.
///   Residual: r = T_i^{-1} * T_j  -  z
///   Analytical Jacobians for both vertices (3x3 each).
class PoseEdge : public Edge {
public:
    PoseEdge() : Edge(3, 2) {}

    void set_measurement(const Vec3& m) { measurement_ = m; }

    void compute_residual() override {
        const auto& pi = vertices_[0]->parameters();
        const auto& pj = vertices_[1]->parameters();

        double dx = pj(0) - pi(0);
        double dy = pj(1) - pi(1);
        double c  = std::cos(pi(2));
        double s  = std::sin(pi(2));

        // transform global displacement into local frame of pose i
        double local_dx     =  c * dx + s * dy;
        double local_dy     = -s * dx + c * dy;
        double local_dtheta = normalize_angle(pj(2) - pi(2));

        residual_(0) = local_dx     - measurement_(0);
        residual_(1) = local_dy     - measurement_(1);
        residual_(2) = normalize_angle(local_dtheta - measurement_(2));
    }

    void compute_jacobians() override {
        const auto& pi = vertices_[0]->parameters();
        const auto& pj = vertices_[1]->parameters();

        double dx = pj(0) - pi(0);
        double dy = pj(1) - pi(1);
        double c  = std::cos(pi(2));
        double s  = std::sin(pi(2));

        // dr / d(pose_i)
        Eigen::Matrix3d Ji;
        Ji << -c, -s, (-s * dx + c * dy),
               s, -c, (-c * dx - s * dy),
               0,  0,  -1;
        jacobians_[0] = Ji;

        // dr / d(pose_j)
        Eigen::Matrix3d Jj;
        Jj <<  c,  s,  0,
              -s,  c,  0,
               0,  0,  1;
        jacobians_[1] = Jj;
    }

    std::string type_info() const override { return "PoseEdge"; }

private:
    Vec3 measurement_ = Vec3::Zero();
};

} // namespace opt
