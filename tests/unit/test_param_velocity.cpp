// ******************************************************
// Unit tests for src/rocket/parameter/velocity.cpp
//   - default ctor zeroes ECI / ECEF / NED / air_body and mach_number
//   - Initialize(datetime, NED, pos_LLH, pos_ECI): NED -> ECEF -> ECI
//   - Update(coordinate, ECI, pos_ECI):            ECI -> ECEF -> NED
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>

#include "Eigen/Dense"

#include "velocity.hpp"
#include "environment/datetime.hpp"
#include "environment/coordinate.hpp"

using forrocket::Velocity;
using forrocket::Coordinate;
using forrocket::DateTime;

// Decision points in velocity.cpp:
//   - Velocity(): zero-inits ECI, ECEF, NED, air_body; mach_number = 0.
//   - Initialize(datetime, NED, pos_LLH, pos_ECI):
//       NED = input; setECEF2NED(pos_LLH); ECEF = NED2ECEF * NED;
//       setECI2ECEF(countup_time); ECI = ECEF2ECI*ECEF + EarthRotate*pos_ECI.
//       SequenceClock(datetime) -> countup_time = 0 -> ECEF2ECI = identity, so
//       ECI = ECEF + EarthRotate * pos_ECI (independent invariant).
//   - Update(coordinate, ECI, pos_ECI):
//       ECEF = ECI2ECEF*ECI - EarthRotate*pos_ECI; NED = ECEF2NED*ECEF.
//   No data-dependent branches; transforms checked against Coordinate's own
//   DCMs (independent references) plus an Initialize<->Update round trip.

TEST(VelocityParam, DefaultConstructorIsZero) {
    Velocity v;
    EXPECT_DOUBLE_EQ(v.ECI.norm(), 0.0);
    EXPECT_DOUBLE_EQ(v.ECEF.norm(), 0.0);
    EXPECT_DOUBLE_EQ(v.NED.norm(), 0.0);
    EXPECT_DOUBLE_EQ(v.air_body.norm(), 0.0);
    EXPECT_DOUBLE_EQ(v.mach_number, 0.0);
}

TEST(VelocityParam, InitializeStoresNEDAndMapsToECEF) {
    Velocity v;
    DateTime t(2019, 1, 1, 0, 0, 0);
    Eigen::Vector3d ned(50.0, -20.0, 5.0);     // north, east, down [m/s]
    Eigen::Vector3d pos_llh(35.0, 139.0, 1000.0);
    Eigen::Vector3d pos_eci(0.0, 0.0, 0.0);    // no Earth-rotation contribution
    v.Initialize(t, ned, pos_llh, pos_eci);

    // NED stored verbatim.
    EXPECT_TRUE(v.NED.isApprox(ned, 1e-12));

    // ECEF must equal NED2ECEF * NED with the DCM built from pos_LLH.
    Coordinate coord;
    coord.setECEF2NED(pos_llh);
    Eigen::Vector3d ecef_ref = coord.dcm.NED2ECEF * ned;
    EXPECT_TRUE(v.ECEF.isApprox(ecef_ref, 1e-9)) << v.ECEF.transpose();

    // With pos_ECI = 0 and epoch-zero (ECEF2ECI = identity), ECI == ECEF.
    EXPECT_TRUE(v.ECI.isApprox(v.ECEF, 1e-9)) << v.ECI.transpose();
}

TEST(VelocityParam, InitializeAddsEarthRotationTerm) {
    // At epoch zero ECEF2ECI = identity, so ECI = ECEF + EarthRotate * pos_ECI.
    Velocity v;
    DateTime t(2019, 1, 1, 0, 0, 0);
    Eigen::Vector3d ned(10.0, 0.0, 0.0);
    Eigen::Vector3d pos_llh(0.0, 0.0, 0.0);
    Eigen::Vector3d pos_eci(7000000.0, 0.0, 0.0);  // 7000 km along ECI x
    v.Initialize(t, ned, pos_llh, pos_eci);

    Coordinate coord;
    coord.setECEF2NED(pos_llh);
    Eigen::Vector3d ecef_ref = coord.dcm.NED2ECEF * ned;
    // EarthRotate = [[0,-w,0],[w,0,0],[0,0,0]]; * (X,0,0) = (0, w*X, 0).
    // Reference: ECI = ECEF + (0, omega * pos_eci_x, 0).
    Eigen::Vector3d eci_ref = ecef_ref + coord.dcm.EarthRotate * pos_eci;
    EXPECT_TRUE(v.ECI.isApprox(eci_ref, 1e-9)) << v.ECI.transpose();
    // Sanity: the y-component picked up omega * 7e6 (~ 0.051 m/s), nonzero.
    EXPECT_GT(std::abs(v.ECI(1) - ecef_ref(1)), 1e-3);
}

TEST(VelocityParam, UpdateMatchesCoordinateTransforms) {
    Coordinate coord;
    coord.setECI2ECEF(98.7);          // non-identity epoch DCM
    coord.setECEF2NED(Eigen::Vector3d(20.0, 50.0, 0.0));  // some site

    Eigen::Vector3d eci(120.0, -30.0, 400.0);
    Eigen::Vector3d pos_eci(6500000.0, 100000.0, 0.0);

    Velocity v;
    v.Update(coord, eci, pos_eci);

    // ECEF = ECI2ECEF * ECI - EarthRotate * pos_ECI (independent reference).
    Eigen::Vector3d ecef_ref =
        coord.dcm.ECI2ECEF * eci - coord.dcm.EarthRotate * pos_eci;
    EXPECT_TRUE(v.ECEF.isApprox(ecef_ref, 1e-9)) << v.ECEF.transpose();
    // NED = ECEF2NED * ECEF.
    Eigen::Vector3d ned_ref = coord.dcm.ECEF2NED * ecef_ref;
    EXPECT_TRUE(v.NED.isApprox(ned_ref, 1e-9)) << v.NED.transpose();
    // ECI stored verbatim.
    EXPECT_TRUE(v.ECI.isApprox(eci, 1e-12));
}

TEST(VelocityParam, InitializeUpdateRoundTripsNED) {
    // Initialize at epoch zero, then Update with a coordinate configured for
    // the same (epoch-zero) rotation and site must recover the original NED.
    Velocity v;
    DateTime t(2019, 1, 1, 0, 0, 0);
    Eigen::Vector3d ned(33.0, -11.0, 7.0);
    Eigen::Vector3d pos_llh(-25.0, 130.0, 200.0);
    Eigen::Vector3d pos_eci(4000000.0, 2000000.0, 1000000.0);
    v.Initialize(t, ned, pos_llh, pos_eci);

    // Reproduce the epoch-zero coordinate that Initialize used internally.
    Coordinate coord;
    coord.setECI2ECEF(0.0);        // countup_time = 0 -> identity rotation
    coord.setECEF2NED(pos_llh);

    Velocity v2;
    v2.Update(coord, v.ECI, pos_eci);

    // The EarthRotate term added in Initialize is removed in Update, and the
    // identity ECI<->ECEF rotation plus the same ECEF2NED matrix recover NED.
    EXPECT_TRUE(v2.NED.isApprox(ned, 1e-7)) << v2.NED.transpose();
}
