// ******************************************************
// Unit tests for src/rocket/parameter/attitude.cpp
//   - default ctor zeroes euler_angle and quaternion
//   - Initialize(euler) builds a unit quaternion from Euler angles
//   - Update(quat, coordinate) normalizes the quaternion and reads the
//     Euler angles back out of the supplied Coordinate
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>

#include "Eigen/Dense"

#include "attitude.hpp"
#include "environment/coordinate.hpp"
#include "degrad.hpp"

using forrocket::Attitude;
using forrocket::Coordinate;
using forrocket::deg2rad;

// Decision points in attitude.cpp:
//   - Attitude(): zero-initializes euler_angle (3) and quaternion (4).
//   - Initialize(euler): sets euler_angle; quaternion = coord.Quaternion(euler).
//   - Update(quat, coord): quaternion = quat.normalized();
//                          euler_angle = coord.EulerAngle().
//   No data-dependent branches inside attitude.cpp itself; tests verify the
//   transforms via frame round-trips (independent references).

TEST(AttitudeParam, DefaultConstructorIsZero) {
    Attitude att;
    EXPECT_DOUBLE_EQ(att.euler_angle(0), 0.0);
    EXPECT_DOUBLE_EQ(att.euler_angle(1), 0.0);
    EXPECT_DOUBLE_EQ(att.euler_angle(2), 0.0);
    for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(att.quaternion(i), 0.0);
}

TEST(AttitudeParam, InitializeProducesUnitQuaternion) {
    Attitude att;
    Eigen::Vector3d euler(deg2rad(30.0), deg2rad(10.0), deg2rad(5.0));  // azi, elv, rol
    att.Initialize(euler);
    // Quaternion() returns a normalized quaternion.
    EXPECT_NEAR(att.quaternion.norm(), 1.0, 1e-12);
    // euler_angle is stored verbatim.
    EXPECT_DOUBLE_EQ(att.euler_angle(0), euler(0));
    EXPECT_DOUBLE_EQ(att.euler_angle(1), euler(1));
    EXPECT_DOUBLE_EQ(att.euler_angle(2), euler(2));
}

TEST(AttitudeParam, InitializeZeroEulerGivesIdentityQuaternion) {
    Attitude att;
    att.Initialize(Eigen::Vector3d(0.0, 0.0, 0.0));
    // Zero Euler -> identity rotation. The DCM is identity, whose largest
    // diagonal term feeds q(3); the convention here yields q = (0,0,0,1).
    EXPECT_NEAR(att.quaternion(0), 0.0, 1e-12);
    EXPECT_NEAR(att.quaternion(1), 0.0, 1e-12);
    EXPECT_NEAR(att.quaternion(2), 0.0, 1e-12);
    EXPECT_NEAR(std::abs(att.quaternion(3)), 1.0, 1e-12);  // sign-agnostic
}

TEST(AttitudeParam, InitializeThenUpdateRoundTripsEuler) {
    // Independent round-trip: Euler -> quaternion (Initialize) -> rebuild the
    // Coordinate DCM from that quaternion -> Update reads Euler back.
    Eigen::Vector3d euler(deg2rad(40.0), deg2rad(-15.0), deg2rad(20.0));
    Attitude att;
    att.Initialize(euler);

    Coordinate coord;
    coord.setNED2Body(att.quaternion);  // rebuild DCM from the unit quaternion
    att.Update(att.quaternion, coord);  // pulls Euler from coord.EulerAngle()

    EXPECT_NEAR(att.euler_angle(0), euler(0), 1e-9) << "azimuth";
    EXPECT_NEAR(att.euler_angle(1), euler(1), 1e-9) << "elevation";
    EXPECT_NEAR(att.euler_angle(2), euler(2), 1e-9) << "roll";
}

TEST(AttitudeParam, UpdateNormalizesQuaternion) {
    // Update must store the *normalized* quaternion even when given a
    // non-unit input. Use a coordinate built from the corresponding rotation.
    Eigen::Vector3d euler(deg2rad(12.0), deg2rad(7.0), deg2rad(3.0));
    Coordinate coord;
    Eigen::Vector4d q_unit = coord.Quaternion(euler);  // already normalized
    coord.setNED2Body(q_unit);

    Eigen::Vector4d q_scaled = q_unit * 5.0;  // non-unit but same orientation
    Attitude att;
    att.Update(q_scaled, coord);

    EXPECT_NEAR(att.quaternion.norm(), 1.0, 1e-12);
    // Normalized scaled quaternion equals the original unit quaternion.
    EXPECT_TRUE(att.quaternion.isApprox(q_unit, 1e-12))
        << att.quaternion.transpose();
}
