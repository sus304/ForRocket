// ******************************************************
// Unit tests for src/environment/vincenty.cpp
//   - vdownrange(): Vincenty inverse geodesic on WGS84
//     returns {downrange [m], azimuth observer->target [deg]}
//
// Reference strategy: independent invariants / known geodesy facts
// rather than characterization wherever possible.
//   * distance(A, A) == 0 (identical-point short-circuit branch)
//   * symmetry: distance(A, B) == distance(B, A)
//   * 1 deg of latitude along a meridian near the equator ~= 110.574 km
//     (WGS84 meridian arc; standard geodesy reference)
//   * azimuth signs: due-north = 0 deg, due-south = +-180 deg, due-east ~ +90
//
// Decision points exercised:
//   D1  identical-point early return (lat==lat && lon==lon)  -> true branch
//   D2  identical-point early return                          -> false branch
//   D3  iterative lambda-fixpoint loop convergence (break)
//   D4  equator-to-equator longitude case: cos_alpha->0 makes
//       cos_2sigma_m = 0/0 -> NaN (degenerate branch documented as
//       _Characterization)
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>
#include <utility>

#include "Eigen/Core"

#include "environment/vincenty.hpp"

using forrocket::vdownrange;

namespace {
// Helper: Eigen LLH [lat deg, lon deg, alt m]
Eigen::Vector3d LLH(double lat, double lon, double alt = 0.0) {
    return Eigen::Vector3d(lat, lon, alt);
}
}  // namespace

// D1: observer == target -> returns {0, 0} via the early-out branch.
TEST(Vincenty, IdenticalPointsReturnsZero) {
    auto r = vdownrange(LLH(35.0, 139.0, 0.0), LLH(35.0, 139.0, 0.0));
    EXPECT_DOUBLE_EQ(r.first, 0.0);   // downrange
    EXPECT_DOUBLE_EQ(r.second, 0.0);  // azimuth
}

// D1 again: altitude differing but lat/lon equal still hits the early-out
// (the guard only compares lat and lon).
TEST(Vincenty, SameLatLonDifferentAltStillZero) {
    auto r = vdownrange(LLH(35.0, 139.0, 0.0), LLH(35.0, 139.0, 5000.0));
    EXPECT_DOUBLE_EQ(r.first, 0.0);
    EXPECT_DOUBLE_EQ(r.second, 0.0);
}

// D2 + D3: a genuine pair runs the iterative solver.
// 1 degree of latitude along the prime meridian, just off the equator.
// WGS84 meridian arc length for 1 deg near the equator is ~110.574 km.
// (Standard geodesy: equatorial degree of latitude ~ 110.574 km.)
TEST(Vincenty, OneDegreeLatitudeMeridianArc) {
    auto r = vdownrange(LLH(0.0, 0.0, 0.0), LLH(1.0, 0.0, 0.0));
    // Source: WGS84 meridian arc; computed value 110574.39 m.
    EXPECT_NEAR(r.first, 110574.39, 1.0);
    // Heading due north -> azimuth 0 deg.
    EXPECT_NEAR(r.second, 0.0, 1e-6);
}

// Symmetry invariant: geodesic distance is direction-independent.
TEST(Vincenty, DistanceIsSymmetric) {
    Eigen::Vector3d A = LLH(35.0, 139.0, 0.0);
    Eigen::Vector3d B = LLH(34.0, 138.0, 0.0);
    auto ab = vdownrange(A, B);
    auto ba = vdownrange(B, A);
    EXPECT_NEAR(ab.first, ba.first, 1e-6) << "distance must be symmetric";
    // Distance is positive for distinct points.
    EXPECT_GT(ab.first, 0.0);
}

// Known longer pair: Tokyo (35.6762,139.6503) -> Osaka (34.6937,135.5023).
// Independent check: great-circle/geodesic distance is ~390-400 km.
// Computed Vincenty value here is 393181.9 m.
TEST(Vincenty, TokyoToOsakaDistance) {
    auto r = vdownrange(LLH(35.6762, 139.6503, 0.0),
                        LLH(34.6937, 135.5023, 0.0));
    // Source: Vincenty inverse on WGS84; ~393.18 km. Sanity band 390-400 km.
    EXPECT_GT(r.first, 390000.0);
    EXPECT_LT(r.first, 400000.0);
    EXPECT_NEAR(r.first, 393181.9, 50.0);
}

// Azimuth conventions: result is atan2-based in (-180, 180].
TEST(Vincenty, AzimuthDueNorthIsZero) {
    auto r = vdownrange(LLH(35.0, 139.0, 0.0), LLH(35.1, 139.0, 0.0));
    EXPECT_NEAR(r.second, 0.0, 1e-4);
    EXPECT_GT(r.first, 0.0);
}

TEST(Vincenty, AzimuthDueSouthIsPlusMinus180) {
    auto r = vdownrange(LLH(35.0, 139.0, 0.0), LLH(34.9, 139.0, 0.0));
    EXPECT_NEAR(std::abs(r.second), 180.0, 1e-4);
}

TEST(Vincenty, AzimuthDueEastNear90) {
    // Small eastward step: azimuth approaches +90 deg.
    auto r = vdownrange(LLH(35.0, 139.0, 0.0), LLH(35.0, 139.1, 0.0));
    EXPECT_NEAR(r.second, 90.0, 0.1);
    EXPECT_GT(r.first, 0.0);
}

// D4: equator-to-equator along a parallel is a degenerate Vincenty case.
// With U1 == U2 == 0 the formula's cos_alpha collapses to 0 and
// cos_2sigma_m = cos_sigma - 2*sin(U1)*sin(U2)/cos_alpha^2 becomes 0/0,
// producing NaN. This documents the *current* numerical behaviour of the
// implementation (no special handling of the equatorial line). It still
// exercises the non-early-out path (D2) and the loop (D3).
TEST(Vincenty, EquatorAlongParallelIsNaN_Characterization) {
    // _Characterization: asserts the present (buggy) NaN output, not a
    // physically correct distance. Documented so a future fix can update it.
    auto r = vdownrange(LLH(0.0, 0.0, 0.0), LLH(0.0, 1.0, 0.0));
    EXPECT_TRUE(std::isnan(r.first));
    EXPECT_TRUE(std::isnan(r.second));
}
