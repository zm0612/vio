//
// Created by meng on 2021/10/6.
//

#ifndef LEAST_SQUARE_EDGE_H
#define LEAST_SQUARE_EDGE_H

#include "vertex.h"

#include <Eigen/Dense>
#include <vector>
#include <memory>

class Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Edge(int residual_dim, int number_vertices);

    virtual void ComputeResidual() = 0;

    virtual void ComputeJacobians() = 0;

    void SetInformation(Eigen::MatrixXd information);

    void AddVertex(const std::shared_ptr<Vertex> &vertex_ptr);

    unsigned int Id() const;

    std::vector<Eigen::MatrixXd> GetJacobian() const;

    double Chi2() const;

    Eigen::VectorXd Chi() const {
        return residual_;
    }

protected:
    Eigen::VectorXd residual_;
    Eigen::MatrixXd information_;
    std::vector<Eigen::MatrixXd> jacobians_;
    std::vector<std::shared_ptr<Vertex>> vertices_;

    unsigned int id_;
};

#endif //LEAST_SQUARE_EDGE_H
