//
// Created by meng on 2021/10/6.
//
#include "edge.h"

unsigned long global_edge_id = 0;

Edge::Edge(int residual_dim, int number_vertices) {
    residual_.resize(residual_dim);
    jacobians_.resize(number_vertices);
    information_.resize(residual_dim, residual_dim);
    information_.setIdentity();

    id_ = global_edge_id++;
}

void Edge::SetInformation(const Eigen::MatrixXd information) {
    information_ = information;
}

unsigned int Edge::Id() const {
    return id_;
}

void Edge::AddVertex(const std::shared_ptr<Vertex> &vertex_ptr) {
    vertices_.emplace_back(vertex_ptr);
}

std::vector<Eigen::MatrixXd> Edge::GetJacobian() const {
    return jacobians_;
}

double Edge::Chi2() const {
    return residual_.squaredNorm();
}