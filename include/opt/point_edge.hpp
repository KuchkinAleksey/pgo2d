#pragma once

#include "edge.hpp"
#include "vertex.hpp"
#include <cmath>
#include <Eigen/Core>

namespace opt {

/// @brief Landmark observation edge: pose -> point
/// @details Vertex 0 = PoseVertex (x, y, theta), Vertex 1 = PointVertex (lx, ly).
///   Measurement z = (dx, dy), landmark position in the local frame of the pose.
///   Residual:  r = R(theta)^T * (landmark - pose_xy)  -  z
///   Analytical Jacobians: 2x3 w.r.t. pose, 2x2 w.r.t. landmark.
class PointEdge : public Edge {
public:
    PointEdge() : Edge(2, 2) {}

    void set_measurement(const Eigen::Vector2d& m) { measurement_ = m; }

    void compute_residual() override {
        const auto& pose  = vertices_[0]->parameters();
        const auto& point = vertices_[1]->parameters();

        double dx = point(0) - pose(0);
        double dy = point(1) - pose(1);
        double c  = std::cos(pose(2));
        double s  = std::sin(pose(2));

        // rotate world-frame displacement into pose-local frame
        residual_(0) =  c * dx + s * dy - measurement_(0);
        residual_(1) = -s * dx + c * dy - measurement_(1);
    }

    void compute_jacobians() override {
        const auto& pose  = vertices_[0]->parameters();
        const auto& point = vertices_[1]->parameters();

        double dx = point(0) - pose(0);
        double dy = point(1) - pose(1);
        double c  = std::cos(pose(2));
        double s  = std::sin(pose(2));

        // dr / d(pose), 2x3
        Eigen::Matrix<double, 2, 3> Jp;
        Jp << -c, -s, (-s * dx + c * dy),
               s, -c, (-c * dx - s * dy);
        jacobians_[0] = Jp;

        // dr / d(landmark), 2x2 (just the rotation matrix)
        Eigen::Matrix2d Jl;
        Jl <<  c,  s,
              -s,  c;
        jacobians_[1] = Jl;
    }

    std::string type_info() const override { return "PointEdge"; }

private:
    Eigen::Vector2d measurement_ = Eigen::Vector2d::Zero();
};

} // namespace opt
