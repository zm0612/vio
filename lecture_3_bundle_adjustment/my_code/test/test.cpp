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
        EXPECT_FLOAT_EQ(least_square_solver_.RobustKernelFunction(1.5), 1.092407891);
        EXPECT_FLOAT_EQ(least_square_solver_.RobustKernelFunctionFirstDerivative(1.5), 0.5466942679);
        EXPECT_FLOAT_EQ(least_square_solver_.RobustKernelFunctionSecondDerivative(1.5), -0.1652130969);
    }
};

TEST_F(TestLeastSquareSolver, TestRobustKernelFunction) {
    TestRobustKernelFunction();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}