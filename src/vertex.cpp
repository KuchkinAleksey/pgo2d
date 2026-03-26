#include <opt/vertex.hpp>

namespace opt {

static unsigned long g_vertex_id = 0;

Vertex::Vertex(int num_dimension, int local_dimension) {
    parameters_.resize(num_dimension, 1);
    local_dimension_ = local_dimension > 0 ? local_dimension : num_dimension;
    id_ = g_vertex_id++;
}

int  Vertex::dimension()       const { return static_cast<int>(parameters_.rows()); }
int  Vertex::local_dimension() const { return local_dimension_; }
void Vertex::plus(const VecX& delta) { parameters_ += delta; }

} // namespace opt
