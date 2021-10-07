//
// Created by meng on 2021/10/6.
//
#include "vertex.h"

unsigned long global_vertex_id = 0;

Vertex::Vertex(const int dim) {
    parameters_.resize(dim);
    id_ = global_vertex_id++;
}

void Vertex::Plus(const Eigen::VectorXd &delta_x) {
    parameters_ += delta_x;
}

void Vertex::SetParameters(const Eigen::VectorXd& parameters) {
    parameters_ = parameters;
}

Eigen::VectorXd Vertex::Parameters() const {
    return parameters_;
}

unsigned int Vertex::Id() const {
    return id_;
}