// ******************************************************
// Unit tests for src/environment/air.cpp
//   forrocket::EnvironmentAir(geometric_altitude)
//
// EnvironmentAir is a thin wrapper around
// standardatmosphere1976::Atmosphere(). It has no internal branches itself; the
// only thing to verify here is that it (a) correctly maps the result array
// fields onto its members and (b) reproduces the US Standard Atmosphere 1976
// reference values at the surface. The branching of the atmosphere model is
// exercised in test_satmo1976.cpp.
//
// Result-array layout (satmo1976::Atmosphere):
//   res[0] = density, res[1] = pressure, res[2] = temperature, res[3] = sound speed
// EnvironmentAir maps:
//   density        <- res[0]
//   pressure       <- res[1]
//   temprature     <- res[2]   (note: member is spelled "temprature")
//   speed_of_sound <- res[3]
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>

#include "environment/air.hpp"
#include "environment/satmo1976.hpp"

using forrocket::EnvironmentAir;

// US Standard Atmosphere 1976 sea-level reference values (these are the
// defining constants of the model, see NOAA/NASA/USAF 1976 doc, and they are
// hard-coded as *_sealevel in satmo1976.cpp):
//   T   = 288.15 K
//   P   = 101325 Pa
//   rho = 1.225 kg/m^3
//   a   = 340.294 m/s  (model uses 340.294; textbook value ~340.29)
TEST(EnvironmentAir, SeaLevelMatchesUSSA1976) {
    EnvironmentAir air(0.0);
    EXPECT_NEAR(air.temprature,     288.15,   1e-6);
    EXPECT_NEAR(air.pressure,       101325.0, 1e-3);
    EXPECT_NEAR(air.density,        1.225,    1e-9);
    EXPECT_NEAR(air.speed_of_sound, 340.294,  1e-6);
}

// Verify the field mapping is exactly the one the satmo model produces, i.e.
// the wrapper does not transpose density<->pressure etc. We recompute the
// underlying array and compare member-by-member.
TEST(EnvironmentAir, MapsResultVectorFieldsCorrectly) {
    const double h = 5000.0;  // troposphere, layer 0
    EnvironmentAir air(h);
    std::array<double, 4> res = standardatmosphere1976::Atmosphere(h);
    ASSERT_EQ(res.size(), 4u);
    EXPECT_DOUBLE_EQ(air.density,        res[0]);
    EXPECT_DOUBLE_EQ(air.pressure,       res[1]);
    EXPECT_DOUBLE_EQ(air.temprature,     res[2]);
    EXPECT_DOUBLE_EQ(air.speed_of_sound, res[3]);
}

// Independent reference at 5 km (troposphere, lapse rate 6.5 K/km on
// geopotential height). Computed from the same formulas the model uses; values
// cross-checked against published US Standard Atmosphere 1976 tables
// (T~255.7 K, P~54048 Pa, rho~0.7364 kg/m^3 at 5 km).
TEST(EnvironmentAir, FiveKilometersTroposphere) {
    EnvironmentAir air(5000.0);
    EXPECT_NEAR(air.temprature, 255.6755,  1e-2);   // K
    EXPECT_NEAR(air.pressure,   54048.3,   1.0);    // Pa
    EXPECT_NEAR(air.density,    0.736429,  1e-4);   // kg/m^3
    // Sound speed = sqrt(T/Tsl)*csl = sqrt(255.6755/288.15)*340.294
    EXPECT_NEAR(air.speed_of_sound, 320.5454, 1e-2);
}

// Invariant: density derived from the model satisfies speed_of_sound > 0 and
// pressure > 0 at all sampled altitudes, and density falls with altitude in the
// lower atmosphere.
TEST(EnvironmentAir, DensityMonotonicDecreaseLowerAtmosphere) {
    double prev = EnvironmentAir(0.0).density;
    for (double h = 1000.0; h <= 80000.0; h += 5000.0) {
        EnvironmentAir air(h);
        EXPECT_GT(air.pressure, 0.0)       << "h=" << h;
        EXPECT_GT(air.speed_of_sound, 0.0) << "h=" << h;
        EXPECT_LT(air.density, prev)       << "h=" << h;
        prev = air.density;
    }
}

// Speed of sound consistency: the model defines
//   a = sqrt(T / T_sealevel) * a_sealevel
// which is equivalent to a = sqrt(gamma*R*T) with gamma*R folded into
// a_sealevel^2 / T_sealevel. Verify the relation holds at an arbitrary
// altitude (20 km).
TEST(EnvironmentAir, SoundSpeedFollowsSqrtTemperatureLaw) {
    EnvironmentAir air(20000.0);
    const double Tsl = 288.15;
    const double csl = 340.294;
    double expected = std::sqrt(air.temprature / Tsl) * csl;
    EXPECT_NEAR(air.speed_of_sound, expected, 1e-9);
}
