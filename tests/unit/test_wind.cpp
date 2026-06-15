// ******************************************************
// Unit tests for src/environment/wind.cpp
//   - EnvironmentWind(bool): constant zero-wind (disabled) construction
//   - EnvironmentWind(std::string): wind profile loaded from a CSV
//   - getNED(altitude): returns [northward, eastward, 0] in NED frame
//
// CSV column convention (see examples/sample_wind.csv):
//     col0 = altitude [m]
//     col1 = u = eastward wind  [m/s]
//     col2 = v = northward wind [m/s]
// getNED maps:  NED(0) = northward(alt) = v
//               NED(1) = eastward(alt)  = u
//               NED(2) = 0
// The InterpolateParameter is built with fill_value "zero", so altitudes
// below the first / above the last table row return 0.
//
// Reference strategy: deterministic in-test CSV fixture, linear-interpolation
// invariants (exact at nodes, midpoint average, zero outside range), and the
// down-mapping order (north vs east).
//
// Decision points exercised:
//   D1  EnvironmentWind(bool) constant-zero branch
//   D2  EnvironmentWind(string) file-load branch
//   D3  interpolation at a table node
//   D4  interpolation between nodes (linear)
//   D5  altitude below table range  -> fill "zero"
//   D6  altitude above table range  -> fill "zero"
//   D7  NED component ordering (north in [0], east in [1], 0 in [2])
// ******************************************************

#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <string>

#include "Eigen/Core"

#include "environment/wind.hpp"

using forrocket::EnvironmentWind;

namespace {

// Writes a small, fully deterministic wind table and returns its path.
// alt  u(east)  v(north)
//  0     0.0      0.0
// 100    10.0    -4.0
// 200    20.0    -8.0   (linear: u = 0.1*alt, v = -0.04*alt over [0,200])
std::string WriteFixtureCsv() {
    std::string path = std::string(std::tmpnam(nullptr)) + "_forrocket_wind.csv";
    std::ofstream ofs(path);
    ofs << "alt,u,v\n";
    ofs << "0,0,0\n";
    ofs << "100,10,-4\n";
    ofs << "200,20,-8\n";
    ofs.close();
    return path;
}

}  // namespace

// D1 + D7: disabled wind is constant zero at any altitude.
TEST(EnvironmentWind, DisabledIsZeroEverywhere) {
    EnvironmentWind wind(false);
    Eigen::Vector3d ned = wind.getNED(0.0);
    EXPECT_DOUBLE_EQ(ned(0), 0.0);
    EXPECT_DOUBLE_EQ(ned(1), 0.0);
    EXPECT_DOUBLE_EQ(ned(2), 0.0);

    Eigen::Vector3d high = wind.getNED(12345.0);
    EXPECT_DOUBLE_EQ(high(0), 0.0);
    EXPECT_DOUBLE_EQ(high(1), 0.0);
    EXPECT_DOUBLE_EQ(high(2), 0.0);
}

// D2 + D3 + D7: at a table node the interpolation returns the tabulated row,
// and getNED places north (v) in [0], east (u) in [1], 0 in [2].
TEST(EnvironmentWind, NodeValuesAndComponentOrder) {
    std::string path = WriteFixtureCsv();
    EnvironmentWind wind(path);

    Eigen::Vector3d ned = wind.getNED(100.0);  // node row: u=10, v=-4
    EXPECT_NEAR(ned(0), -4.0, 1e-9) << "NED north component must be v";
    EXPECT_NEAR(ned(1), 10.0, 1e-9) << "NED east component must be u";
    EXPECT_DOUBLE_EQ(ned(2), 0.0)  << "NED down component is always 0";

    std::remove(path.c_str());
}

// D4: linear interpolation halfway between two nodes (alt=150).
// u = 0.1*150 = 15, v = -0.04*150 = -6.
TEST(EnvironmentWind, LinearInterpolationMidpoint) {
    std::string path = WriteFixtureCsv();
    EnvironmentWind wind(path);

    Eigen::Vector3d ned = wind.getNED(150.0);
    EXPECT_NEAR(ned(1), 15.0, 1e-9) << "east (u) interpolated";   // halfway 10..20
    EXPECT_NEAR(ned(0), -6.0, 1e-9) << "north (v) interpolated";  // halfway -4..-8

    std::remove(path.c_str());
}

// D5: altitude below the table range -> fill_value "zero".
TEST(EnvironmentWind, BelowRangeIsZero) {
    std::string path = WriteFixtureCsv();
    EnvironmentWind wind(path);

    Eigen::Vector3d ned = wind.getNED(-50.0);
    EXPECT_DOUBLE_EQ(ned(0), 0.0);
    EXPECT_DOUBLE_EQ(ned(1), 0.0);
    EXPECT_DOUBLE_EQ(ned(2), 0.0);

    std::remove(path.c_str());
}

// D6: altitude above the table range -> fill_value "zero".
TEST(EnvironmentWind, AboveRangeIsZero) {
    std::string path = WriteFixtureCsv();
    EnvironmentWind wind(path);

    Eigen::Vector3d ned = wind.getNED(10000.0);
    EXPECT_DOUBLE_EQ(ned(0), 0.0);
    EXPECT_DOUBLE_EQ(ned(1), 0.0);
    EXPECT_DOUBLE_EQ(ned(2), 0.0);

    std::remove(path.c_str());
}

// Ground node is exactly zero wind (table row alt=0 -> u=v=0).
TEST(EnvironmentWind, GroundNodeIsZero) {
    std::string path = WriteFixtureCsv();
    EnvironmentWind wind(path);

    Eigen::Vector3d ned = wind.getNED(0.0);
    EXPECT_NEAR(ned(0), 0.0, 1e-9);
    EXPECT_NEAR(ned(1), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(ned(2), 0.0);

    std::remove(path.c_str());
}
