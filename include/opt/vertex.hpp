#pragma once

#include "types.hpp"
#include <string>

namespace opt {

/// @brief Base class for optimization graph vertices
/// @details Each vertex holds a parameter vector and an ordering index
/// used to place its block in the Hessian matrix.
class Vertex {
public:
    explicit Vertex(int num_dimension, int local_dimension = -1);
    virtual ~Vertex() = default;

    int           dimension()       const;
    int           local_dimension() const;
    unsigned long id()              const { return id_; }

    const VecX& parameters() const { return parameters_; }
    VecX&       parameters()       { return parameters_; }
    void set_parameters(const VecX& p)  { parameters_ = p; }

    /// @brief Manifold update: p <- p (+) delta
    virtual void        plus(const VecX& delta);
    virtual std::string type_info() const = 0;

    int  ordering_id()             const { return ordering_id_; }
    void set_ordering_id(int id)         { ordering_id_ = id; }

    bool is_fixed()                const { return fixed_; }
    void set_fixed(bool f = true)        { fixed_ = f; }

protected:
    VecX          parameters_;
    int           local_dimension_;
    unsigned long id_;
    int           ordering_id_ = 0;
    bool          fixed_       = false;
};

} // namespace opt
