// ******************************************************
// Unit tests for src/rocket/parameter/force.cpp
//   - Force default constructor zero-inits thrust/aero/gravity
//   - Force::Sum() == thrust + aero + gravity (vector sum)
// ******************************************************

#include <gtest/gtest.h>

#include "Eigen/Core"

#include "force.hpp"

using forrocket::Force;

// Decision points in force.cpp:
//   - Constructor: zeroes thrust, aero, gravity (no branches).
//   - Sum(): single return of thrust + aero + gravity (no branches).
//   Tests cover the zero state and the additive behavior, including an
//   invariant (Sum equals the sum of components for arbitrary vectors).

TEST(ForceParam, DefaultConstructorIsZero) {
    Force f;
    EXPECT_DOUBLE_EQ(f.thrust.norm(), 0.0);
    EXPECT_DOUBLE_EQ(f.aero.norm(), 0.0);
    EXPECT_DOUBLE_EQ(f.gravity.norm(), 0.0);
    // Sum of three zero vectors is the zero vector.
    EXPECT_DOUBLE_EQ(f.Sum().norm(), 0.0);
}

TEST(ForceParam, SumEqualsComponentSum) {
    Force f;
    f.thrust  << 100.0,   0.0,  0.0;
    f.aero    <<  -5.0,   2.0, -1.0;
    f.gravity <<   0.0,   0.0, -9.8;
    // Reference computed component-wise from the three vectors above:
    //   x: 100 + (-5) + 0   = 95
    //   y:   0 +   2  + 0   =  2
    //   z:   0 + (-1) + (-9.8) = -10.8
    Eigen::Vector3d sum = f.Sum();
    EXPECT_DOUBLE_EQ(sum(0), 95.0);
    EXPECT_DOUBLE_EQ(sum(1), 2.0);
    EXPECT_DOUBLE_EQ(sum(2), -10.8);
    // Invariant: Sum() must equal the direct component sum (independent ref).
    EXPECT_TRUE(sum.isApprox(f.thrust + f.aero + f.gravity, 1e-15));
}

TEST(ForceParam, SumIsCommutativeUnderReassignment) {
    // Swapping which physical force lives in which slot must not change Sum,
    // since addition is commutative.
    Force a, b;
    a.thrust  << 1.0, 2.0, 3.0;
    a.aero    << 4.0, 5.0, 6.0;
    a.gravity << 7.0, 8.0, 9.0;
    b.thrust  << 7.0, 8.0, 9.0;
    b.aero    << 1.0, 2.0, 3.0;
    b.gravity << 4.0, 5.0, 6.0;
    EXPECT_TRUE(a.Sum().isApprox(b.Sum(), 1e-15));
}
