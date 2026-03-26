#pragma once

#include "vertex.hpp"

namespace opt {

/// @brief 2D landmark vertex: (x, y)
class PointVertex : public Vertex {
public:
    PointVertex() : Vertex(2) {}
    std::string type_info() const override { return "PointVertex"; }
};

} // namespace opt
