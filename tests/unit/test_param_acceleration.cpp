// ******************************************************
// Unit tests for src/rocket/parameter/acceleration.cpp
//   - Acceleration default constructor zero-initializes ECI and body
// ******************************************************

#include <gtest/gtest.h>

#include "Eigen/Core"

#include "acceleration.hpp"

using forrocket::Acceleration;

// Decision points in acceleration.cpp:
//   The class has only a default constructor that sets ECI and body to zero.
//   No branches; the test verifies the documented zero initialization.

TEST(AccelerationParam, DefaultConstructorIsZero) {
    Acceleration a;
    EXPECT_TRUE(a.ECI.isApprox(Eigen::Vector3d::Zero(), 0.0))
        << "ECI = " << a.ECI.transpose();
    EXPECT_TRUE(a.body.isApprox(Eigen::Vector3d::Zero(), 0.0))
        << "body = " << a.body.transpose();
    // isApprox returns false for an exact-zero reference; check explicitly too.
    EXPECT_DOUBLE_EQ(a.ECI(0), 0.0);
    EXPECT_DOUBLE_EQ(a.ECI(1), 0.0);
    EXPECT_DOUBLE_EQ(a.ECI(2), 0.0);
    EXPECT_DOUBLE_EQ(a.body(0), 0.0);
    EXPECT_DOUBLE_EQ(a.body(1), 0.0);
    EXPECT_DOUBLE_EQ(a.body(2), 0.0);
}

TEST(AccelerationParam, FieldsAreMutable) {
    // The members are public data; confirm they can be assigned independently.
    Acceleration a;
    a.ECI << 1.0, 2.0, 3.0;
    a.body << -4.0, 5.0, -6.0;
    EXPECT_DOUBLE_EQ(a.ECI(0), 1.0);
    EXPECT_DOUBLE_EQ(a.ECI(2), 3.0);
    EXPECT_DOUBLE_EQ(a.body(0), -4.0);
    EXPECT_DOUBLE_EQ(a.body(2), -6.0);
}
