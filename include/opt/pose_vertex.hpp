#pragma once

#include "vertex.hpp"
#include <cmath>

namespace opt {

/// @brief 2D pose vertex: (x, y, theta)
/// @details Overrides plus() to wrap theta into [-pi, pi] after each update.
class PoseVertex : public Vertex {
public:
    PoseVertex() : Vertex(3) {}

    void plus(const VecX& delta) override {
        parameters_ += delta;
        parameters_(2) = normalize_angle(parameters_(2));
    }

    std::string type_info() const override { return "PoseVertex"; }
};

} // namespace opt
