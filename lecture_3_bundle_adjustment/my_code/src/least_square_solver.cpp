//
// Created by meng on 2021/10/6.
//
#include "tic_toc.h"
#include "least_square_solver.h"
#include <cmath>

void LeastSquareSolver::AddVertex(const std::shared_ptr<Vertex> &vertex) {
    if (vertices_.find(vertex->Id()) != vertices_.end()) {
        LOG(FATAL) << "Has same id of vertex";
    } else {
        vertices_.emplace(vertex->Id(), vertex);
    }
}

void LeastSquareSolver::AddEdge(const std::shared_ptr<Edge> &edge) {
    if (edges_.find(edge->Id()) != edges_.end()) {
        LOG(FATAL) << "Has same id of edge";
    } else {
        edges_.emplace(edge->Id(), edge);
    }
}

Eigen::MatrixXd LeastSquareSolver::ComputeW(double chi2, Eigen::VectorXd f) {
    Eigen::MatrixXd I;
    I.resize(f.rows(), f.rows());
    I.setIdentity();

    Eigen::MatrixXd W;
    W = RobustKernelFunctionFirstDerivative(chi2) * I;
    W = 2 * RobustKernelFunctionSecondDerivative(chi2) * f * f.transpose() + W;

    return W;
}

void LeastSquareSolver::SolveLinearEquation() {
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Hessian_.rows(), Hessian_.cols());
    delta_x_ = (Hessian_ + current_mu_ * I).inverse() * b_;
//    delta_x_ = Hessian_.inverse() * b_;
}

void LeastSquareSolver::InitLM() {
    v_ = 2.0;
    current_chi2_ = 0.0;
    for (const auto &edge: edges_) {
        current_chi2_ += edge.second->Chi2();
    }

    double max_value = 0.;
    for (int i = 0; i < Hessian_.cols(); ++i) {
        max_value = std::max(fabs(Hessian_(i, i)), max_value);
    }

    constexpr double tau = 1e-5;
    current_mu_ = tau * max_value;
}

void LeastSquareSolver::ComputeHessian() {
    unsigned int order = vertices_.begin()->second->GetOrder();
    Hessian_.resize(order, order);
    Hessian_.setZero();
    b_.resize(order);
    b_.setZero();

    for (auto &edge: edges_) {
        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();
        Eigen::MatrixXd J_i = edge.second->GetJacobian()[0];
#ifdef USE_ROBUST_KERNEL
        Hessian_ += J_i.transpose() * ComputeW(edge.second->Chi2(), edge.second->Chi()) * J_i;
        b_ += -RobustKernelFunctionFirstDerivative(edge.second->Chi2()) * J_i.transpose() * edge.second->Chi();
#else
        Hessian_ += J_i.transpose() * J_i;
        b_ -= J_i.transpose() * edge.second->Chi();
#endif
    }
}

double LeastSquareSolver::RobustKernelFunction(double s) const {
    return c_ * c_ * std::log10(1.0 + s / (c_ * c_));
}

double LeastSquareSolver::RobustKernelFunctionFirstDerivative(double s) const {
    return 1.0 / (1.0 + s / (c_ * c_));
}

double LeastSquareSolver::RobustKernelFunctionSecondDerivative(double s) {
    return -1.0 / (c_ * c_) * std::pow(RobustKernelFunctionFirstDerivative(s), 2);
}

void LeastSquareSolver::UpdateState() {
    for (auto &vertex: vertices_) {
        vertex.second->Plus(delta_x_);
    }
}

bool LeastSquareSolver::CheckLM() {
    double L_value = 0.001;
    L_value += delta_x_.transpose() * (current_mu_ * delta_x_ + b_);

    double temp_chi2 = 0.0;
    for (auto &edge: edges_) {
        edge.second->ComputeResidual();
        temp_chi2 += edge.second->Chi2();
    }

    double rho = (current_chi2_ - temp_chi2) / L_value;

    if (rho > 0 && std::isfinite(temp_chi2)) {
        double temp = 1.0 - std::pow(2 * rho - 1, 3);
        temp = std::min(temp, 2.0 / 3.0);
        current_mu_ *= std::max(1.0 / 3.0, temp);
        v_ = 2.0;
        current_chi2_ = temp_chi2;
        return true;
    } else {
        current_mu_ *= v_;
        v_ *= 2;
        return false;
    }
}

void LeastSquareSolver::RollBackState() {
    for (auto &vertex: vertices_) {
        vertex.second->Plus(-delta_x_);
    }
}

bool LeastSquareSolver::Solve(const int iterations) {
    TicToc tic_toc;
    ComputeHessian();
    InitLM();

    unsigned int false_count = 0u;
    for (int i = 0; i < iterations; ++i) {
        SolveLinearEquation();

        std::cout << "iter: " << i << " , chi= " << current_chi2_ << " , mu= " << current_mu_ << std::endl;
        if (delta_x_.squaredNorm() <= 1e-6)
            break;

        UpdateState();

        while (true) {
            if (false_count > 10u) {
                return false;
            }

            if (!CheckLM()) {
                RollBackState();
                SolveLinearEquation();
                UpdateState();
                false_count++;
            } else {
                ComputeHessian();
                false_count = 0u;
                break;
            }
        }
    }
    std::cout << "Optimization use time: " << tic_toc.toc() << " ms" << std::endl;

    return true;
}