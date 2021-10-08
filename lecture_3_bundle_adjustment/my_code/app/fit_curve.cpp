//
// Created by meng on 2021/10/7.
//
#include "least_square_solver.h"
#include "vertex.h"
#include "edge.h"

#include <glog/logging.h>
#include <iostream>
#include <random>

class FitCurveVertex : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FitCurveVertex() : Vertex(3) {}
};

class FitCurveEdge : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    __attribute__((unused))
    FitCurveEdge(double x, double y) : Edge(1, 1) {
        x_ = x;
        y_ = y;
    }

    void ComputeJacobians() override {
        Eigen::Vector3d abc = vertices_.at(0)->Parameters();
        double y = std::exp(abc[0] * x_ * x_ + abc[1] * x_ + abc[2]);
        Eigen::Matrix<double, 1, 3> jacobian;
        jacobian << y * x_ * x_, y * x_, y * 1;
        jacobians_[0] = jacobian;
    }

    void ComputeResidual() override {
        Eigen::Vector3d abc = vertices_.at(0)->Parameters();
        residual_[0] = std::exp(abc[0] * x_ * x_ + abc[1] * x_ + abc[2]) - y_;
    }

private:
    double x_;
    double y_;
};

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;

    double a = 1.5, b = 2.8, c = 1.2;
    int N = 300;
    double w_sigma = 0.5;

    std::default_random_engine random_engine;
    std::normal_distribution<double> noise(0, w_sigma);

    LeastSquareSolver least_square_solver;

    std::shared_ptr<FitCurveVertex> vertex_ptr;
    vertex_ptr = std::make_shared<FitCurveVertex>();
    vertex_ptr->SetParameters(Eigen::Vector3d(0, 0, 0));
    least_square_solver.AddVertex(vertex_ptr);

    for (int i = 0; i < N; ++i) {
        double x = i * 1.0 / N;
        double n = noise(random_engine);
        double y = std::exp(a * x * x + b * x + c) + n;

        std::shared_ptr<FitCurveEdge> edge_ptr;
        edge_ptr = std::make_shared<FitCurveEdge>(x, y);
        edge_ptr->AddVertex(vertex_ptr);
        least_square_solver.AddEdge(edge_ptr);
    }

    least_square_solver.Solve(30);


    std::cout << "-------After optimization--------" << std::endl;
    std::cout << vertex_ptr->Parameters().transpose() << std::endl;
    std::cout << "----------ground truth-----------" << std::endl;
    std::cout << "        1.5,  2.8,  1.2   " << std::endl;

    return 0;
}