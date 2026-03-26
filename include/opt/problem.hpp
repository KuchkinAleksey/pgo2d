#pragma once

#include "types.hpp"
#include "edge.hpp"
#include "vertex.hpp"

#include <functional>
#include <map>
#include <memory>
#include <unordered_map>

namespace opt {

/// Called after each successful LM iteration: (iteration, chi2, lambda)
using SolveCallback = std::function<void(int, double, double)>;

class Problem {
public:
    using VertexMap = std::map<unsigned long, std::shared_ptr<Vertex>>;
    using EdgeMap   = std::unordered_map<unsigned long, std::shared_ptr<Edge>>;

    bool add_vertex(std::shared_ptr<Vertex> vertex);
    bool remove_vertex(std::shared_ptr<Vertex> vertex);
    bool add_edge(std::shared_ptr<Edge> edge);
    bool remove_edge(std::shared_ptr<Edge> edge);

    /// @brief Run Levenberg-Marquardt optimization
    /// @param max_iterations Upper bound on outer iterations
    /// @param callback Invoked after each accepted step with (iter, chi2, lambda)
    /// @return true if the solver ran (even if it hit the iteration limit)
    bool solve(int max_iterations, SolveCallback callback = nullptr);

    double current_chi() const { return current_chi_; }

private:
    /// Assign ordering indices to all vertices for Hessian assembly
    void set_ordering();

    /// Build the normal equations: H = sum(J^T * Omega * J), b = -sum(J^T * Omega * r)
    void make_hessian();

    /// Solve (H + lambda*I) * dx = b via LDLT
    void solve_linear_system();

    /// Apply delta_x_ to all non-fixed vertices
    void update_states();

    /// Undo the last update_states() (subtract delta_x_)
    void rollback_states();

    /// Set initial lambda from max diagonal of H (Nielsen strategy)
    void compute_lambda_init();

    /// Evaluate gain ratio rho and update lambda accordingly
    /// @return true if the step reduced chi2
    bool is_good_step();

    MatXX  hessian_;
    VecX   b_;
    VecX   delta_x_;

    double current_lambda_  = 0;
    double current_chi_     = 0;
    double stop_threshold_  = 0;
    double ni_              = 2.0;   ///< Nielsen damping factor

    VertexMap vertices_;
    EdgeMap   edges_;
    std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> vertex_to_edge_;
    unsigned long ordering_generic_ = 0;
};

} // namespace opt
