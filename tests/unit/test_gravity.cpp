// ******************************************************
// Unit tests for inc/environment/gravity.hpp
//   forrocket::gravity(altitude) — Newtonian point-mass gravity using the
//   WGS84 GM and semi-major axis.
//
// Model under test (gravity.hpp):
//   geocentric_height = (altitude < 0) ? a : altitude + a
//   g = GM / geocentric_height^2
//
// Decision points covered:
//   D1  altitude < 0.0   -> clamp geocentric_height to wgs84.a
//   D2  else (altitude >= 0.0) -> geocentric_height = altitude + a
// Plus invariants: monotonic decrease with altitude, continuity at 0,
// and agreement with an independent GM/r^2 reference.
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>

#include "environment/gravity.hpp"
#include "environment/wgs84.hpp"

using forrocket::gravity;
using forrocket::WGS84;

// Independent reference: surface value is GM/a^2 with the WGS84 *defining*
// constants GM = 3.986004418e14 m^3/s^2 and a = 6378137.0 m.
//   3.986004418e14 / 6378137.0^2 = 9.7982854791873 m/s^2
// (This is the bare GM/a^2; it is intentionally NOT 9.80665 m/s^2 because the
// model omits Earth rotation and the J2 oblateness term.)
static const double kG_surface_ref = 3.986004418e14 / (6378137.0 * 6378137.0);

// D2: altitude == 0 -> g = GM / a^2. Tolerance 1e-9 since this is an exact
// closed-form evaluation in double precision.
TEST(Gravity, SurfaceEqualsGMoverA2) {
    EXPECT_NEAR(gravity(0.0), kG_surface_ref, 1e-9);
    // Sanity: surface gravity is ~9.80 m/s^2.
    EXPECT_NEAR(gravity(0.0), 9.798285, 1e-5);
}

// D2: positive altitude uses geocentric_height = altitude + a.
// At 100 km: GM / (a + 1.0e5)^2 computed independently.
TEST(Gravity, At100kmMatchesGMoverR2) {
    WGS84 wgs;
    double r = wgs.a + 100.0e3;
    double expected = wgs.GM / (r * r);  // independent GM/r^2
    EXPECT_NEAR(gravity(100.0e3), expected, 1e-12);
    // Independent numeric check (python: 9.498117091820768).
    EXPECT_NEAR(gravity(100.0e3), 9.498117091820768, 1e-9);
}

// D1: any negative altitude is clamped to the surface value, so g is constant
// and equal to gravity(0) for all altitude < 0.
TEST(Gravity, NegativeAltitudeClampsToSurface) {
    EXPECT_DOUBLE_EQ(gravity(-100.0), gravity(0.0));
    EXPECT_DOUBLE_EQ(gravity(-5000.0), gravity(0.0));
    EXPECT_DOUBLE_EQ(gravity(-1.0e6), gravity(0.0));
    // And it equals the GM/a^2 reference.
    EXPECT_NEAR(gravity(-100.0), kG_surface_ref, 1e-9);
}

// Boundary between D1 and D2 is altitude == 0.0 (the < is strict, so 0 takes
// the else branch). The function must be continuous there: gravity(0) from the
// else branch equals the clamp value.
TEST(Gravity, ContinuousAtZeroBoundary) {
    double from_below = gravity(-1e-9);  // D1 clamp
    double at_zero    = gravity(0.0);    // D2, but altitude+a == a
    EXPECT_DOUBLE_EQ(from_below, at_zero);
}

// Invariant: gravity strictly decreases with increasing altitude (inverse
// square law), for altitudes >= 0.
TEST(Gravity, MonotonicDecreaseWithAltitude) {
    double prev = gravity(0.0);
    for (double h = 1.0e3; h <= 1000.0e3; h += 50.0e3) {
        double g = gravity(h);
        EXPECT_LT(g, prev) << "altitude " << h;
        prev = g;
    }
}

// Invariant: doubling the geocentric radius quarters the acceleration.
// Choose altitude = a so geocentric_height = 2a -> g = GM/(2a)^2 = g0/4.
TEST(Gravity, InverseSquareScaling) {
    WGS84 wgs;
    double g0 = gravity(0.0);
    double g_at_2a = gravity(wgs.a);  // geocentric_height = 2a
    EXPECT_NEAR(g_at_2a, g0 / 4.0, 1e-12);
}
