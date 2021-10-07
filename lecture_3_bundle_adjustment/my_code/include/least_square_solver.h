//
// Created by meng on 2021/10/6.
//

#ifndef LEAST_SQUARE_LEAST_SQUARE_SOLVER_H
#define LEAST_SQUARE_LEAST_SQUARE_SOLVER_H

#include "vertex.h"
#include "edge.h"

#include <glog/logging.h>
#include <Eigen/Dense>

#include <unordered_map>
#include <memory>
#include <vector>
#include <iostream>

class LeastSquareSolver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    friend class TestLeastSquareSolver;

    LeastSquareSolver() = default;

    ~LeastSquareSolver() = default;

    bool Solve(const int iterations);

    void AddVertex(const std::shared_ptr<Vertex>& vertex);

    void AddEdge(const std::shared_ptr<Edge> &edge);

private:
    void RollBackState();

    bool CheckLM();

    void UpdateState();

    void SolveLinearEquation();

    void InitLM();

    Eigen::MatrixXd ComputeW(double chi2, Eigen::VectorXd f);

    void ComputeHessian();

    double RobustKernelFunction(double s) const;

    double RobustKernelFunctionFirstDerivative(double s) const;

    double RobustKernelFunctionSecondDerivative(double s);

private:
    std::unordered_map<unsigned int, std::shared_ptr<Vertex>> vertices_;
    std::unordered_map<unsigned int, std::shared_ptr<Edge>> edges_;

    Eigen::MatrixXd Hessian_;
    Eigen::MatrixXd Jacobian_;
    Eigen::VectorXd b_;
    Eigen::VectorXd delta_x_;

    double current_chi2_;
    double current_mu_;
    double v_;
    const double c_ = 1.345;
};

#endif //LEAST_SQUARE_LEAST_SQUARE_SOLVER_H
