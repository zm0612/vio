//
// Created by meng on 2021/10/6.
//
#include "least_square_solver.h"

#include <gtest/gtest.h>

class TestLeastSquareSolver : public ::testing::Test {
public:
    LeastSquareSolver least_square_solver_;

    void SetUp() override {
    }

    void TearDown() override {

    }

    void TestRobustKernelFunction() {
        EXPECT_FLOAT_EQ(least_square_solver_.RobustKernelFunction(2.0), 0.7442918359);
        EXPECT_FLOAT_EQ(least_square_solver_.RobustKernelFunctionFirstDerivative(2.0), 0.7398457931);
        EXPECT_FLOAT_EQ(least_square_solver_.RobustKernelFunctionSecondDerivative(2.0), -0.0962369982);
    }

    void TestRobustKernelFunctionCoefficient() {
        double chi2 = 2.0;
        Eigen::Matrix<double, 1,1> f(std::sqrt(2.0));
        std::cout << least_square_solver_.ComputeW(chi2, f) << std::endl;
    }
};

TEST_F(TestLeastSquareSolver, TestRobustKernelFunction) {
    TestRobustKernelFunction();
    TestRobustKernelFunctionCoefficient();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}