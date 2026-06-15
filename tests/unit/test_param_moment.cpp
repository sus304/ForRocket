// ******************************************************
// Unit tests for src/rocket/parameter/moment.cpp
//   - Moment default constructor zero-inits all six contributions
//   - Moment::Sum() == gyro + thrust + aero_force + aero_dumping
//                      + jet_dumping + gas_jet (vector sum)
// ******************************************************

#include <gtest/gtest.h>

#include "Eigen/Core"

#include "moment.hpp"

using forrocket::Moment;

// Decision points in moment.cpp:
//   - Constructor: zeroes gyro, thrust, aero_force, aero_dumping,
//     jet_dumping, gas_jet (no branches).
//   - Sum(): single return summing the six vectors (no branches).
//   Tests cover the zero state, additive behavior with a hand-computed
//   reference, and the invariant Sum == direct component sum.

TEST(MomentParam, DefaultConstructorIsZero) {
    Moment m;
    EXPECT_DOUBLE_EQ(m.gyro.norm(), 0.0);
    EXPECT_DOUBLE_EQ(m.thrust.norm(), 0.0);
    EXPECT_DOUBLE_EQ(m.aero_force.norm(), 0.0);
    EXPECT_DOUBLE_EQ(m.aero_dumping.norm(), 0.0);
    EXPECT_DOUBLE_EQ(m.jet_dumping.norm(), 0.0);
    EXPECT_DOUBLE_EQ(m.gas_jet.norm(), 0.0);
    EXPECT_DOUBLE_EQ(m.Sum().norm(), 0.0);
}

TEST(MomentParam, SumEqualsComponentSum) {
    Moment m;
    m.gyro         << 1.0, 0.0, 0.0;
    m.thrust       << 0.0, 1.0, 0.0;
    m.aero_force   << 0.0, 0.0, 1.0;
    m.aero_dumping << 2.0, 0.0, 0.0;
    m.jet_dumping  << 0.0, 3.0, 0.0;
    m.gas_jet      << 0.0, 0.0, 4.0;
    // Reference computed component-wise:
    //   x: 1 + 0 + 0 + 2 + 0 + 0 = 3
    //   y: 0 + 1 + 0 + 0 + 3 + 0 = 4
    //   z: 0 + 0 + 1 + 0 + 0 + 4 = 5
    Eigen::Vector3d sum = m.Sum();
    EXPECT_DOUBLE_EQ(sum(0), 3.0);
    EXPECT_DOUBLE_EQ(sum(1), 4.0);
    EXPECT_DOUBLE_EQ(sum(2), 5.0);
    // Invariant: Sum() equals the direct six-term sum (independent ref).
    EXPECT_TRUE(sum.isApprox(
        m.gyro + m.thrust + m.aero_force + m.aero_dumping + m.jet_dumping + m.gas_jet,
        1e-15));
}

TEST(MomentParam, SingleContributionPassesThrough) {
    // With only one contribution non-zero, Sum() must equal that contribution.
    Moment m;
    m.aero_dumping << -1.5, 2.5, -3.5;
    EXPECT_TRUE(m.Sum().isApprox(Eigen::Vector3d(-1.5, 2.5, -3.5), 1e-15));
}
