//
// Created by meng on 2021/10/6.
//

#ifndef LEAST_SQUARE_VERTEX_H
#define LEAST_SQUARE_VERTEX_H

#include <Eigen/Dense>

class Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit Vertex(int dim);

    ~Vertex() = default;

    void Plus(const Eigen::VectorXd &delta);

    void SetParameters(const Eigen::VectorXd &parameters);

    Eigen::VectorXd Parameters() const;

    unsigned int GetOrder() const {
        return parameters_.rows();
    }

    unsigned int Id() const;

protected:
    Eigen::VectorXd parameters_;
    unsigned int id_;
};

#endif //LEAST_SQUARE_VERTEX_H
