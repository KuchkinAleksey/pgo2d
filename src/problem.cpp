#include <opt/problem.hpp>

#include <cassert>
#include <cmath>
#include <iostream>

namespace opt {

bool Problem::add_vertex(std::shared_ptr<Vertex> vertex) {
    if (vertices_.contains(vertex->id())) return false;
    vertices_[vertex->id()] = vertex;
    return true;
}

bool Problem::remove_vertex(std::shared_ptr<Vertex> vertex) {
    if (!vertices_.contains(vertex->id())) return false;

    // remove all edges that touch this vertex
    auto range = vertex_to_edge_.equal_range(vertex->id());
    for (auto it = range.first; it != range.second; ++it)
        edges_.erase(it->second->id());

    vertex_to_edge_.erase(vertex->id());
    vertices_.erase(vertex->id());
    return true;
}

bool Problem::add_edge(std::shared_ptr<Edge> edge) {
    if (edges_.contains(edge->id())) return false;
    edges_[edge->id()] = edge;
    for (auto& v : edge->vertices())
        vertex_to_edge_.emplace(v->id(), edge);
    return true;
}

bool Problem::remove_edge(std::shared_ptr<Edge> edge) {
    if (!edges_.contains(edge->id())) return false;
    edges_.erase(edge->id());
    return true;
}

bool Problem::solve(int max_iterations, SolveCallback callback) {
    if (edges_.empty() || vertices_.empty()) {
        std::cerr << "Cannot solve: no edges or vertices\n";
        return false;
    }

    set_ordering();
    make_hessian();
    compute_lambda_init();

    // report initial state before any updates
    if (callback) callback(0, current_chi_, current_lambda_);

    bool stop = false;
    int  iter = 0;

    while (!stop && iter < max_iterations) {
        bool one_step_success = false;
        int  false_cnt        = 0;

        // inner loop: try to find a lambda that gives a descent step
        while (!one_step_success) {
            solve_linear_system();

            if (delta_x_.squaredNorm() <= 1e-6 || false_cnt > 10) {
                stop = true;
                break;
            }

            update_states();
            one_step_success = is_good_step();

            if (one_step_success) {
                make_hessian();
                false_cnt = 0;
            } else {
                ++false_cnt;
                rollback_states();
            }
        }
        ++iter;

        if (callback) callback(iter, current_chi_, current_lambda_);

        if (std::sqrt(current_chi_) <= stop_threshold_)
            stop = true;
    }
    return true;
}

void Problem::set_ordering() {
    ordering_generic_ = 0;
    for (auto& [id, vertex] : vertices_) {
        vertex->set_ordering_id(static_cast<int>(ordering_generic_));
        ordering_generic_ += vertex->local_dimension();
    }
}

void Problem::make_hessian() {
    auto size = ordering_generic_;
    MatXX H = MatXX::Zero(size, size);
    VecX  b = VecX::Zero(size);

    for (auto& [eid, edge] : edges_) {
        edge->compute_residual();
        edge->compute_jacobians();

        auto& jacobians = edge->jacobians();
        auto& verts     = edge->vertices();
        assert(jacobians.size() == verts.size());

        for (size_t i = 0; i < verts.size(); ++i) {
            auto& v_i = verts[i];
            if (v_i->is_fixed()) continue;

            auto& J_i = jacobians[i];
            auto  idx_i = v_i->ordering_id();
            auto  dim_i = v_i->local_dimension();

            // J_i^T * Omega, reused for both H and b contributions
            MatXX JtW = J_i.transpose() * edge->information();

            for (size_t j = i; j < verts.size(); ++j) {
                auto& v_j = verts[j];
                if (v_j->is_fixed()) continue;

                auto& J_j = jacobians[j];
                auto  idx_j = v_j->ordering_id();
                auto  dim_j = v_j->local_dimension();

                MatXX hessian = JtW * J_j;
                H.block(idx_i, idx_j, dim_i, dim_j).noalias() += hessian;
                if (j != i)
                    H.block(idx_j, idx_i, dim_j, dim_i).noalias() += hessian.transpose();
            }
            b.segment(idx_i, dim_i).noalias() -= JtW * edge->residual();
        }
    }

    hessian_ = H;
    b_       = b;
    delta_x_ = VecX::Zero(size);
}

void Problem::solve_linear_system() {
    MatXX H = hessian_;
    // add LM damping to diagonal
    for (unsigned long i = 0; i < static_cast<unsigned long>(hessian_.cols()); ++i)
        H(i, i) += current_lambda_;
    delta_x_ = H.ldlt().solve(b_);
}

void Problem::update_states() {
    for (auto& [id, vertex] : vertices_) {
        if (vertex->is_fixed()) continue;
        int idx = vertex->ordering_id();
        int dim = vertex->local_dimension();
        vertex->plus(delta_x_.segment(idx, dim));
    }
}

void Problem::rollback_states() {
    for (auto& [id, vertex] : vertices_) {
        if (vertex->is_fixed()) continue;
        int idx = vertex->ordering_id();
        int dim = vertex->local_dimension();
        vertex->plus(-delta_x_.segment(idx, dim));
    }
}

void Problem::compute_lambda_init() {
    ni_             = 2.0;
    current_lambda_ = -1.0;
    current_chi_    = 0.0;

    for (auto& [id, edge] : edges_)
        current_chi_ += edge->chi2();

    stop_threshold_ = 1e-6 * current_chi_;

    // initial lambda from the largest diagonal element of H (tau * max_diag)
    double max_diag = 0.0;
    for (unsigned long i = 0; i < static_cast<unsigned long>(hessian_.cols()); ++i)
        max_diag = std::max(std::abs(hessian_(i, i)), max_diag);

    current_lambda_ = 1e-5 * max_diag;
}

bool Problem::is_good_step() {
    // gain ratio denominator: predicted reduction from the linear model
    double scale = (delta_x_.transpose() * (current_lambda_ * delta_x_ + b_))(0);
    scale += 1e-3;  // guard against division by zero

    // recompute chi2 at the new linearization point
    double temp_chi = 0.0;
    for (auto& [id, edge] : edges_) {
        edge->compute_residual();
        temp_chi += edge->chi2();
    }

    // rho = actual_reduction / predicted_reduction
    double rho = (current_chi_ - temp_chi) / scale;

    if (rho > 0.0 && std::isfinite(temp_chi)) {
        // good step: decrease lambda (Nielsen update rule)
        double alpha = 1.0 - std::pow(2.0 * rho - 1.0, 3);
        alpha = std::min(alpha, 2.0 / 3.0);
        current_lambda_ *= std::max(1.0 / 3.0, alpha);
        ni_          = 2.0;
        current_chi_ = temp_chi;
        return true;
    } else {
        // bad step: increase lambda, will retry with more damping
        current_lambda_ *= ni_;
        ni_ *= 2.0;
        return false;
    }
}

} // namespace opt
