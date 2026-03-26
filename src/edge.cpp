#include <opt/edge.hpp>
#include <opt/vertex.hpp>

namespace opt {

static unsigned long g_edge_id = 0;

Edge::Edge(int residual_dimension, int num_vertices) {
    residual_.resize(residual_dimension, 1);
    jacobians_.resize(num_vertices);
    id_ = g_edge_id++;

    information_.resize(residual_dimension, residual_dimension);
    information_.setIdentity();
}

double Edge::chi2() const {
    return (residual_.transpose() * information_ * residual_)(0);
}

} // namespace opt
