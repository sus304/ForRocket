// ******************************************************
// Unit tests for src/rocket/parameter/position.cpp
//   - default ctor zeroes ECI / ECEF / LLH
//   - Initialize(datetime, LLH): LLH -> ECEF -> ECI
//   - Update(coordinate, ECI):   ECI -> ECEF -> LLH
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>

#include "Eigen/Dense"

#include "position.hpp"
#include "environment/datetime.hpp"
#include "environment/coordinate.hpp"
#include "environment/wgs84.hpp"

using forrocket::Position;
using forrocket::Coordinate;
using forrocket::DateTime;
using forrocket::WGS84;

// Decision points in position.cpp:
//   - Position(): zero-inits ECI, ECEF, LLH (no branches).
//   - Initialize(datetime, LLH): LLH = input; ECEF = LLH2ECEF(LLH);
//       setECI2ECEF(clock.countup_time); ECI = ECEF2ECI * ECEF.
//       SequenceClock(datetime) sets countup_time = 0, so ECI2ECEF = identity
//       => ECI == ECEF (independent invariant, no fixture needed).
//   - Update(coordinate, ECI): ECEF = ECI2ECEF * ECI; LLH = ECEF2LLH(ECEF).
//   No data-dependent branches inside position.cpp; transforms are checked
//   against Coordinate's own conversions (independent references).

TEST(PositionParam, DefaultConstructorIsZero) {
    Position p;
    EXPECT_DOUBLE_EQ(p.ECI.norm(), 0.0);
    EXPECT_DOUBLE_EQ(p.ECEF.norm(), 0.0);
    EXPECT_DOUBLE_EQ(p.LLH.norm(), 0.0);
}

TEST(PositionParam, InitializeStoresLLHAndMatchesECEF) {
    Position p;
    DateTime t(2019, 1, 1, 0, 0, 0);
    Eigen::Vector3d llh(35.0, 139.0, 1000.0);  // lat[deg], lon[deg], h[m]
    p.Initialize(t, llh);

    // LLH stored verbatim.
    EXPECT_DOUBLE_EQ(p.LLH(0), 35.0);
    EXPECT_DOUBLE_EQ(p.LLH(1), 139.0);
    EXPECT_DOUBLE_EQ(p.LLH(2), 1000.0);

    // ECEF must equal Coordinate::LLH2ECEF(LLH) (independent reference).
    Coordinate coord;
    Eigen::Vector3d ecef_ref = coord.LLH2ECEF(llh);
    EXPECT_TRUE(p.ECEF.isApprox(ecef_ref, 1e-6)) << p.ECEF.transpose();
}

TEST(PositionParam, InitializeAtEpochZeroGivesECIEqualsECEF) {
    // SequenceClock(datetime) -> countup_time = 0 -> setECI2ECEF(0) is the
    // identity rotation, hence ECEF2ECI = identity and ECI == ECEF.
    Position p;
    DateTime t(2020, 6, 15, 12, 30, 0);
    Eigen::Vector3d llh(0.0, 0.0, 0.0);  // equator, prime meridian, sea level
    p.Initialize(t, llh);

    // At the equator/prime-meridian/sea-level, ECEF x = semi-major axis a,
    // y = z = 0 (matches WGS84 a).
    WGS84 wgs;
    EXPECT_NEAR(p.ECEF(0), wgs.a, 1e-6);
    EXPECT_NEAR(p.ECEF(1), 0.0, 1e-6);
    EXPECT_NEAR(p.ECEF(2), 0.0, 1e-6);

    // Epoch-zero invariant: ECI == ECEF.
    EXPECT_TRUE(p.ECI.isApprox(p.ECEF, 1e-9)) << p.ECI.transpose();
}

TEST(PositionParam, UpdateRoundTripsThroughCoordinate) {
    // Build a coordinate at a non-trivial epoch, then verify Update applies
    // ECI2ECEF and ECEF2LLH exactly as Coordinate does.
    Coordinate coord;
    coord.setECI2ECEF(123.456);  // arbitrary epoch time [s] -> non-identity DCM

    // Start from a known LLH, map to ECEF (frame-fixed), then to ECI via the
    // inverse rotation, so the values are mutually consistent.
    Eigen::Vector3d llh(-10.0, 200.0, 5000.0);
    Eigen::Vector3d ecef = coord.LLH2ECEF(llh);
    Eigen::Vector3d eci = coord.dcm.ECEF2ECI * ecef;

    Position p;
    p.Update(coord, eci);

    // ECEF = ECI2ECEF * ECI must recover the original ECEF.
    EXPECT_TRUE(p.ECEF.isApprox(ecef, 1e-6)) << p.ECEF.transpose();
    // LLH must match Coordinate::ECEF2LLH(ECEF) and round-trip the input LLH.
    Eigen::Vector3d llh_ref = coord.ECEF2LLH(ecef);
    EXPECT_NEAR(p.LLH(0), llh_ref(0), 1e-7);
    EXPECT_NEAR(p.LLH(1), llh_ref(1), 1e-7);
    EXPECT_NEAR(p.LLH(2), llh_ref(2), 1e-4);
    // ECI stored verbatim.
    EXPECT_TRUE(p.ECI.isApprox(eci, 1e-9));
}
