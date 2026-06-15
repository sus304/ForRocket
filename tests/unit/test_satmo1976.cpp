// ******************************************************
// Unit tests for src/environment/satmo1976.cpp
//   US Standard Atmosphere 1976 model (namespace standardatmosphere1976).
//
// Public API exercised:
//   std::vector<double> Atmosphere(double geometric_altitude)   // [m] in
//        returns {density, pressure, temperature, sound_speed}
//   std::vector<double> LowerAtmosphere(double geometric_altitude_km)
//        returns {density, pressure, temperature}
//   std::vector<double> UpperAtmosphere(double geometric_altitude_km)
//        returns {density, pressure, temperature}
//   double EvaluateCubic(a,fa,fpa, b,fb,fpb, u)  // Hermite-style interpolant
//   double KineticTemperature(double geometric_altitude_km)
//
// Decision points covered (see per-function comments below):
//   Atmosphere:        geometric_altitude > 86 km  (UpperAtmosphere) vs else
//                      (LowerAtmosphere).
//   LowerAtmosphere:   each layer index i selected by the binary search
//                      (layers 0..6 reachable; layer 7 is the top boundary),
//                      and the temp_gradient == 0 branch vs the != 0 branch.
//   UpperAtmosphere:   geometric_altitude > 1000 km clamp branch vs the
//                      binary-search interpolation branch; the log/cubic loop.
//   KineticTemperature: all four branches (z<=91, 91<z<110, 110<=z<120,
//                      z>=120) plus the boundary equalities.
//   EvaluateCubic:     endpoint identities and linear reproduction.
//
// IMPORTANT FINDING (documented, not tested): UpperAtmosphere reads
// height_array[i+1]. At *exactly* 1000 km the clamp ( > 1000 km ) is false, the
// binary search lands on i=24, and the code dereferences height_array[25] which
// is out of bounds (std::array<double,25>). Tests therefore stay strictly below
// 1000 km on the interpolation path and use altitude > 1000 km for the clamp.
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "environment/satmo1976.hpp"

namespace sa = standardatmosphere1976;

// ---------------------------------------------------------------------------
// Module-level constants (defining values of US Standard Atmosphere 1976).
// ---------------------------------------------------------------------------
static const double kTsl   = 288.15;     // sea-level temperature [K]
static const double kPsl   = 101325.0;   // sea-level pressure [Pa]
static const double kRhosl = 1.225;      // sea-level density [kg/m^3]
static const double kCsl   = 340.294;    // sea-level sound speed [m/s]

// ===========================================================================
//  Module constants
// ===========================================================================
TEST(Satmo1976, ModuleConstantsMatchUSSA1976) {
    // Defining constants of the 1976 standard atmosphere.
    EXPECT_DOUBLE_EQ(sa::temp_sealevel,     288.15);
    EXPECT_DOUBLE_EQ(sa::pressure_sealevel, 101325.0);
    EXPECT_DOUBLE_EQ(sa::density_sealevel,  1.225);
    EXPECT_DOUBLE_EQ(sa::soundspeed_sealevel, 340.294);
    EXPECT_DOUBLE_EQ(sa::g0, 9.80665);
    EXPECT_DOUBLE_EQ(sa::mol_weight_sealevel, 28.9644);
    EXPECT_DOUBLE_EQ(sa::Rstar, 8314.32);
    // gmr = 1000 * g0 * M / R* = 1000*9.80665*28.9644/8314.32 = 34.16319...
    EXPECT_NEAR(sa::gmr, 34.163194736310366, 1e-9);
    // earth_radius aliases the polar radius (6356.7523 km) in this model.
    EXPECT_DOUBLE_EQ(sa::earth_radius, 6356.7523);
}

// ===========================================================================
//  Atmosphere() dispatch  (decision: geometric_altitude > 86 km ?)
// ===========================================================================

// Atmosphere() returns 4 elements; res[3] = sqrt(T/Tsl)*csl.
TEST(Satmo1976, AtmosphereReturnsFourElementsWithSoundSpeed) {
    std::vector<double> r = sa::Atmosphere(0.0);
    ASSERT_EQ(r.size(), 4u);
    // res[2] is temperature; res[3] must equal sqrt(res[2]/Tsl)*csl.
    EXPECT_NEAR(r[3], std::sqrt(r[2] / kTsl) * kCsl, 1e-9);
}

// Else branch: altitude <= 86 km routes to LowerAtmosphere. At sea level the
// model reproduces its own defining sea-level constants exactly (theta=1,
// delta=1, sigma=1).
TEST(Satmo1976, AtmosphereSeaLevelIsLowerBranch) {
    std::vector<double> r = sa::Atmosphere(0.0);
    EXPECT_NEAR(r[0], kRhosl, 1e-9);   // density
    EXPECT_NEAR(r[1], kPsl,   1e-3);   // pressure
    EXPECT_NEAR(r[2], kTsl,   1e-9);   // temperature
    EXPECT_NEAR(r[3], kCsl,   1e-6);   // sound speed
}

// If branch: altitude > 86 km routes to UpperAtmosphere. At 86.001 km we are
// just above the boundary; temperature is still T7 = 186.8673 K (KineticT
// z<=91 branch) and pressure is ~ the 86 km node value.
TEST(Satmo1976, AtmosphereAbove86kmIsUpperBranch) {
    std::vector<double> r = sa::Atmosphere(86001.0);  // 86.001 km
    EXPECT_NEAR(r[2], 186.8673, 1e-3);  // KineticTemperature(86.001) == T7
    EXPECT_GT(r[1], 0.0);
    EXPECT_GT(r[0], 0.0);
    EXPECT_NEAR(r[3], std::sqrt(r[2] / kTsl) * kCsl, 1e-9);
}

// Boundary: exactly 86 km uses the else (Lower) branch because the test is
// strictly ">". Lower at 86 km geometric (geopotential 84.852 km) hits the top
// layer (i=6 region) and is finite/positive.
TEST(Satmo1976, AtmosphereAt86kmUsesLowerBranch) {
    std::vector<double> r = sa::Atmosphere(86000.0);
    // T at 86 km ~ 186.9 K (geopotential 84.852 km, layer with gradient -2 K/km
    // evaluated up to its top). Independent calc gives ~186.946 K.
    EXPECT_NEAR(r[2], 186.946, 0.2);
    EXPECT_GT(r[1], 0.0);
}

// ===========================================================================
//  LowerAtmosphere()  (units: km in, {rho,P,T} out)
//  Covers every layer the binary search can select, the lapse-rate decisions,
//  and the temp_gradient == 0 (isothermal) branch.
// ===========================================================================

// Layer 0 (0..11 km, gradient -6.5 K/km, NON-zero gradient branch).
// Reference at sea level (geopotential 0): theta=delta=sigma=1.
TEST(Satmo1976, LowerLayer0SeaLevel) {
    std::vector<double> r = sa::LowerAtmosphere(0.0);
    EXPECT_NEAR(r[0], kRhosl, 1e-12);
    EXPECT_NEAR(r[1], kPsl,   1e-6);
    EXPECT_NEAR(r[2], kTsl,   1e-12);
}

// Layer 0 lapse rate: temperature decreases at 6.5 K/km of *geopotential*
// height. At geometric 5 km the geopotential height is ~4.9961 km, so
// T = 288.15 - 6.5*4.9961 = 255.675 K. (Cross-checked vs USSA-76 table ~255.7.)
TEST(Satmo1976, LowerLayer0LapseRate) {
    std::vector<double> r = sa::LowerAtmosphere(5.0);
    EXPECT_NEAR(r[2], 255.6755, 1e-2);   // K
    EXPECT_NEAR(r[1], 54048.3,  1.0);    // Pa
    EXPECT_NEAR(r[0], 0.736429, 1e-4);   // kg/m^3
}

// Layer 1 (11..20 km, gradient 0 -> ISOTHERMAL branch: exp form).
// Tropopause temperature is the famous 216.65 K, constant across the layer.
TEST(Satmo1976, LowerLayer1IsothermalTropopause) {
    std::vector<double> r15 = sa::LowerAtmosphere(15.0);
    std::vector<double> r20 = sa::LowerAtmosphere(20.0);
    EXPECT_NEAR(r15[2], 216.65, 1e-2);   // K, isothermal
    EXPECT_NEAR(r20[2], 216.65, 1e-2);   // K, isothermal
    // Pressure still decreases with altitude in the isothermal layer.
    EXPECT_LT(r20[1], r15[1]);
    // Independent values from the model formulas (USSA-76).
    EXPECT_NEAR(r15[1], 12111.8, 5.0);   // Pa
    EXPECT_NEAR(r20[1], 5529.31, 2.0);   // Pa
}

// Layer 2 (20..32 km, gradient +1 K/km -> temperature INCREASES, non-zero
// branch). At geometric 25 km, geopotential ~24.90 km, T ~221.55 K.
TEST(Satmo1976, LowerLayer2PositiveGradient) {
    std::vector<double> r = sa::LowerAtmosphere(25.0);
    EXPECT_NEAR(r[2], 221.5521, 1e-2);
    EXPECT_GT(r[2], 216.65);  // warmer than the isothermal layer below
}

// Layer 3 (32..47 km, gradient +2.8 K/km). At geometric 40 km, T ~250.35 K.
TEST(Satmo1976, LowerLayer3) {
    std::vector<double> r = sa::LowerAtmosphere(40.0);
    EXPECT_NEAR(r[2], 250.3496, 1e-2);
}

// Layer 4 (47..51 km, gradient 0 -> ISOTHERMAL again, T = 270.65 K).
TEST(Satmo1976, LowerLayer4IsothermalStratopause) {
    std::vector<double> r = sa::LowerAtmosphere(49.0);
    EXPECT_NEAR(r[2], 270.65, 1e-2);  // K (stratopause isothermal)
}

// Layer 5 (51..71 km, gradient -2.8 K/km, temperature decreasing).
TEST(Satmo1976, LowerLayer5NegativeGradient) {
    std::vector<double> r = sa::LowerAtmosphere(60.0);
    EXPECT_NEAR(r[2], 247.0209, 1e-2);
}

// Layer 6 (71..84.852 km, gradient -2.0 K/km). At geometric 80 km, T ~198.64 K.
TEST(Satmo1976, LowerLayer6) {
    std::vector<double> r = sa::LowerAtmosphere(80.0);
    EXPECT_NEAR(r[2], 198.6386, 1e-2);
}

// Invariant across the whole lower atmosphere: pressure decreases monotonically
// with altitude (a fundamental property of any hydrostatic atmosphere).
TEST(Satmo1976, LowerPressureMonotonicDecrease) {
    double prev = sa::LowerAtmosphere(0.0)[1];
    for (double z = 1.0; z <= 84.0; z += 1.0) {
        double p = sa::LowerAtmosphere(z)[1];
        EXPECT_LT(p, prev) << "z=" << z << " km";
        prev = p;
    }
}

// ===========================================================================
//  UpperAtmosphere()  (units: km in, {rho,P,T} out)
//  Covers the clamp branch and the interpolation branch + node reproduction.
// ===========================================================================

// Clamp branch: geometric_altitude > 1000 km returns the last-node ratios and
// a flat temperature of 1000 K. Use 1100 km so the strict ">" is satisfied.
// Last node density ratio = 2.907e-15, pressure ratio = 7.4155e-14.
TEST(Satmo1976, UpperClampAbove1000km) {
    std::vector<double> r = sa::UpperAtmosphere(1100.0);
    EXPECT_NEAR(r[2], 1000.0, 1e-9);                 // flat top temperature
    EXPECT_NEAR(r[0], 2.907e-15 * kRhosl, 1e-25);    // density = dr_last*rhosl
    EXPECT_NEAR(r[1], 7.4155e-14 * kPsl,  1e-20);    // pressure = pr_last*Psl
}

// Interpolation branch, node reproduction: EvaluateCubic at u == left node
// returns exactly the left node value, so at an exact table altitude the model
// reproduces the tabulated ratios. 200 km is node index 14:
//   pressure ratio = 8.3628e-10, density ratio = 2.074e-10.
// Temperature at 200 km comes from KineticTemperature (z>=120 branch).
TEST(Satmo1976, UpperNodeReproductionAt200km) {
    std::vector<double> r = sa::UpperAtmosphere(200.0);
    EXPECT_NEAR(r[1], 8.3628e-10 * kPsl,  1e-15);  // pressure
    EXPECT_NEAR(r[0], 2.074e-10  * kRhosl, 1e-16); // density
    // KineticTemperature(200) ~ 854.56 K (independent calc).
    EXPECT_NEAR(r[2], 854.5590852363246, 1e-3);
}

// Interpolation branch, between nodes: at 100 km the model lands on node index
// 2 (the search picks i with height[i] <= z). 100 km is itself a node
// (index 2), so ratios reproduce the table: pr=3.1593e-7, dr=4.575e-7.
TEST(Satmo1976, UpperNodeReproductionAt100km) {
    std::vector<double> r = sa::UpperAtmosphere(100.0);
    EXPECT_NEAR(r[1], 3.1593e-7 * kPsl,  1e-7);
    EXPECT_NEAR(r[0], 4.575e-7  * kRhosl, 1e-10);
}

// Invariant: pressure decreases monotonically with altitude through the upper
// atmosphere as well (sampled strictly below the 1000 km out-of-bounds point).
TEST(Satmo1976, UpperPressureMonotonicDecrease) {
    double prev = sa::UpperAtmosphere(90.0)[1];
    for (double z = 100.0; z <= 990.0; z += 10.0) {
        double p = sa::UpperAtmosphere(z)[1];
        EXPECT_LT(p, prev) << "z=" << z << " km";
        prev = p;
    }
}

// ===========================================================================
//  KineticTemperature()  (all four branches + boundaries)
// ===========================================================================

// Branch 1: z <= 91 km -> constant T7 = 186.8673 K. Includes the boundary
// z == 91 (the test is "<=").
TEST(Satmo1976, KineticTempBranch1Below91) {
    EXPECT_NEAR(sa::KineticTemperature(86.0), 186.8673, 1e-9);
    EXPECT_NEAR(sa::KineticTemperature(91.0), 186.8673, 1e-9);  // boundary z==Z8
}

// Branch 2: 91 < z < 110 -> elliptical segment TC + C1*sqrt(1-((z-Z8)/C2)^2).
// At z = 100: xx=(100-91)/19.9429=0.45129, T = 263.1905 - 76.3232*sqrt(1-xx^2)
//           = 195.0813 K (independent calc).
TEST(Satmo1976, KineticTempBranch2Elliptical) {
    EXPECT_NEAR(sa::KineticTemperature(100.0), 195.08134433524688, 1e-6);
}

// Branch 3: 110 <= z < 120 -> linear T9 + C3*(z-Z9) = 240 + 12*(z-110).
// Includes boundary z == 110 (Z9): falls out of branch 2 (z<Z9 false) into
// branch 3, giving exactly T9 = 240 K.
TEST(Satmo1976, KineticTempBranch3LinearAndZ9Boundary) {
    EXPECT_NEAR(sa::KineticTemperature(110.0), 240.0, 1e-9);  // boundary z==Z9
    EXPECT_NEAR(sa::KineticTemperature(115.0), 300.0, 1e-9);  // 240+12*5
}

// Branch 4: z >= 120 -> exospheric exponential approach to T12 = 1000 K.
// Includes boundary z == 120 (Z10): branch 3 (z<Z10) false -> else, giving
// exactly T10 = 360 K.
TEST(Satmo1976, KineticTempBranch4ExponentialAndZ10Boundary) {
    EXPECT_NEAR(sa::KineticTemperature(120.0), 360.0, 1e-9);  // boundary z==Z10
    // z = 200: independent calc gives 854.559... K.
    EXPECT_NEAR(sa::KineticTemperature(200.0), 854.5590852363246, 1e-6);
    // As z grows the temperature asymptotically approaches T12 = 1000 K and
    // stays below it.
    double t_high = sa::KineticTemperature(900.0);
    EXPECT_LT(t_high, 1000.0);
    EXPECT_GT(t_high, 360.0);
}

// Monotonic increase across branches 3->4 region (temperature rises toward the
// exosphere).
TEST(Satmo1976, KineticTempIncreasesAbove110) {
    double prev = sa::KineticTemperature(110.0);
    for (double z = 115.0; z <= 900.0; z += 25.0) {
        double t = sa::KineticTemperature(z);
        EXPECT_GT(t, prev) << "z=" << z;
        prev = t;
    }
}

// ===========================================================================
//  EvaluateCubic()  (Hermite-style interpolant identities)
// ===========================================================================

// Endpoint identity at u == a: the interpolant returns fa exactly (the t and
// p*t terms vanish at t=0).
TEST(Satmo1976, EvaluateCubicEndpointA) {
    double v = sa::EvaluateCubic(0.0, 2.0, 0.5, 10.0, 7.0, -0.3, 0.0);
    EXPECT_NEAR(v, 2.0, 1e-12);
}

// Endpoint identity at u == b: returns fb exactly (p = 0 at t=1).
TEST(Satmo1976, EvaluateCubicEndpointB) {
    double v = sa::EvaluateCubic(0.0, 2.0, 0.5, 10.0, 7.0, -0.3, 10.0);
    EXPECT_NEAR(v, 7.0, 1e-12);
}

// Linear reproduction: for a straight line f(x) = x (so fa=a, fb=b, fpa=fpb=1,
// slope d=1 == fpa == fpb), the correction term (d-fpa)/(d-fpb) is zero and the
// interpolant is exactly linear: f(5) = 5.
TEST(Satmo1976, EvaluateCubicReproducesLine) {
    double v = sa::EvaluateCubic(0.0, 0.0, 1.0, 10.0, 10.0, 1.0, 5.0);
    EXPECT_NEAR(v, 5.0, 1e-12);
}
