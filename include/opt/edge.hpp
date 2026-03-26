#pragma once

#include "types.hpp"
#include <memory>
#include <string>
#include <vector>

namespace opt {

class Vertex;

/// @brief Base class for optimization graph edges (constraints / factors)
/// @details Connects one or more vertices. Subclasses implement
/// compute_residual() and compute_jacobians() for specific cost models.
class Edge {
public:
    explicit Edge(int residual_dimension, int num_vertices);
    virtual ~Edge() = default;

    unsigned long id() const { return id_; }

    void set_vertices(const std::vector<std::shared_ptr<Vertex>>& v) { vertices_ = v; }
    void add_vertex(std::shared_ptr<Vertex> v) { vertices_.push_back(std::move(v)); }

    std::shared_ptr<Vertex>                       vertex(int i)  const { return vertices_[i]; }
    const std::vector<std::shared_ptr<Vertex>>&   vertices()     const { return vertices_; }
    size_t                                        num_vertices() const { return vertices_.size(); }

    virtual std::string type_info()        const = 0;
    virtual void compute_residual()              = 0;
    virtual void compute_jacobians()             = 0;

    /// @brief Mahalanobis cost: r^T * Omega * r
    double chi2() const;

    const VecX&              residual()    const { return residual_; }
    const std::vector<MatXX>& jacobians()  const { return jacobians_; }

    void        set_information(const MatXX& info) { information_ = info; }
    const MatXX& information()                const { return information_; }

protected:
    unsigned long                          id_;
    std::vector<std::shared_ptr<Vertex>>   vertices_;
    VecX                                   residual_;
    std::vector<MatXX>                     jacobians_;
    MatXX                                  information_;  ///< inverse covariance
};

} // namespace opt
