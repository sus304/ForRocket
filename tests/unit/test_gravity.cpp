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

#include "degrad.hpp"
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

// ---------------------------------------------------------------------------
// gravityECEF(): point-mass + J2 zonal harmonic (attraction only, ECEF frame).
// Validated against WGS84 Somigliana normal gravity, which includes the
// centrifugal acceleration — so the test adds it back before comparing.
// Point-mass+J2 truncates J4 and higher; the residual peaks at the pole at
// ~1.2e-4 m/s^2 (~1.2e-5 g), hence the 2e-4 m/s^2 gate (measured: equator
// 2e-5, 45deg 5e-5, pole 1.18e-4).
// ---------------------------------------------------------------------------
#include "Eigen/Core"

namespace {

// WGS84 Somigliana normal gravity on the ellipsoid surface [m/s2].
double SomiglianaGravity(double lat_rad) {
    const double gamma_e = 9.7803253359;
    const double k = 0.00193185265241;
    const double e_sq = 0.00669437999014;
    double s2 = std::sin(lat_rad) * std::sin(lat_rad);
    return gamma_e * (1.0 + k * s2) / std::sqrt(1.0 - e_sq * s2);
}

// Geodetic latitude (h=0, lon=0) -> ECEF surface position.
Eigen::Vector3d SurfaceECEF(double lat_rad) {
    const double a = 6378137.0;
    const double e_sq = 0.00669437999014;
    double N = a / std::sqrt(1.0 - e_sq * std::sin(lat_rad) * std::sin(lat_rad));
    return Eigen::Vector3d(N * std::cos(lat_rad), 0.0, N * (1.0 - e_sq) * std::sin(lat_rad));
}

}  // namespace

TEST(GravityECEF, MatchesSomiglianaNormalGravityAt0_45_90deg) {
    const double omega = 7.292115e-5;
    const double lats_deg[] = {0.0, 45.0, 90.0};
    for (double lat_deg : lats_deg) {
        double lat = lat_deg * forrocket::pi / 180.0;
        Eigen::Vector3d r = SurfaceECEF(lat);
        Eigen::Vector3d g_attraction = forrocket::gravityECEF(r);
        // Normal gravity = attraction + centrifugal (centrifugal points outward
        // from the spin axis; as apparent "gravity" contribution it reduces g).
        Eigen::Vector3d centrifugal(omega * omega * r(0), omega * omega * r(1), 0.0);
        double g_apparent = (g_attraction + centrifugal).norm();
        EXPECT_NEAR(g_apparent, SomiglianaGravity(lat), 2e-4)
                << "latitude " << lat_deg << " deg";
    }
}

TEST(GravityECEF, EquatorIsExactlyPointMassTimesJ2Factor) {
    // At the equator sin(geocentric lat) = 0, so the x-component reduces to
    // GM/r^2 * (1 + 1.5*J2*(a/r)^2) with r = a.
    forrocket::WGS84 wgs84;
    Eigen::Vector3d r(wgs84.a, 0.0, 0.0);
    Eigen::Vector3d g = forrocket::gravityECEF(r);
    EXPECT_NEAR(g(0), -wgs84.GM / (wgs84.a * wgs84.a) * (1.0 + 1.5 * wgs84.J2), 1e-12);
    EXPECT_DOUBLE_EQ(g(1), 0.0);
    EXPECT_DOUBLE_EQ(g(2), 0.0);
}

TEST(GravityECEF, PointsInwardAndDecaysWithAltitude) {
    forrocket::WGS84 wgs84;
    Eigen::Vector3d r_low = SurfaceECEF(45.0 * forrocket::pi / 180.0);
    Eigen::Vector3d r_high = r_low * ((r_low.norm() + 500.0e3) / r_low.norm());
    Eigen::Vector3d g_low = forrocket::gravityECEF(r_low);
    Eigen::Vector3d g_high = forrocket::gravityECEF(r_high);
    EXPECT_LT(g_low.dot(r_low), 0.0);   // inward
    EXPECT_LT(g_high.norm(), g_low.norm());  // decays
    // 500 km up the J2-relative contribution shrinks (a/r)^2-fold.
    EXPECT_NEAR(g_high.norm(), wgs84.GM / std::pow(r_high.norm(), 2), 2e-2);
}
