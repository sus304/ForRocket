// ******************************************************
// Unit tests for src/factory/engine_factory.cpp
//
// EngineFactory::Create reads a JSON config file and builds an Engine via one
// of FOUR branch combinations:
//   {Enable Thrust File: true | false} x {Enable Engine Miss Alignment: true | false}
//
//   1. thrust-file  + no misalignment
//   2. thrust-file  + misalignment
//   3. constant     + no misalignment
//   4. constant     + misalignment
//
// All four are covered below (one TEST each). The thrust-file branches exercise
// forrocket::LoadCsvLog (a temp CSV with a header row + data rows is written).
//
// PATHS: the test CWD is the build dir, so the JSON path passed to Create() and
// the "Thrust at vacuum File Path" stored inside the JSON are both ABSOLUTE
// (/tmp/...). Temp files are removed at the end of each test.
// ******************************************************

#include <gtest/gtest.h>

#include <cmath>      // std::pow
#include <cstdio>     // std::remove
#include <cstdlib>    // EXIT_FAILURE
#include <fstream>
#include <string>

#include "factory/engine_factory.hpp"

using forrocket::Engine;
using forrocket::EngineFactory;

namespace {

// --- shared numeric constants -----------------------------------------------
// degrad.hpp defines pi = 3.141592653589793; engine_factory uses it for area.
const double kPi = 3.141592653589793;
// The factory's deg->rad conversion now uses degrad.hpp's full-precision pi
// (the truncated 3.14159265 literal was fixed in the accuracy batch).
const double kPiTrunc = kPi;

const double kDiameterMm = 100.0;                  // "Nozzle Exit Diameter [mm]"
const double kDiameterM = kDiameterMm / 1e3;       // 0.1 m
// area_exit = 0.25 * pi * D^2 with D in metres = 0.25 * pi * 0.1^2.
const double kAreaExit = 0.25 * std::pow(kDiameterM, 2) * kPi;

// Temp file paths (absolute, under /tmp).
const char* kJsonPath = "/tmp/forrocket_test_engine_factory.json";
const char* kCsvPath = "/tmp/forrocket_test_thrust.csv";

// Writes a 3-column thrust CSV with a header line (LoadCsvLog default
// skip_rows=1). Columns: time, thrust[N], mdot[kg/s].
void WriteThrustCsv(const std::string& path) {
    std::ofstream ofs(path);
    ofs << "t,thrust,mdot\n"   // header (skipped by LoadCsvLog)
        << "0,1000,1\n"
        << "10,1000,1\n";       // burn_duration = last time = 10
    ofs.close();
}

// Writes an engine config JSON. enable_thrust_file / enable_misalign select the
// branch; the thrust-file path points at the (absolute) CSV path.
void WriteEngineJson(const std::string& path, bool enable_thrust_file,
                     bool enable_misalign, const std::string& csv_abs_path) {
    std::ofstream ofs(path);
    ofs << "{\n"
        << "    \"Nozzle Exit Diameter [mm]\": " << kDiameterMm << ",\n"
        << "    \"Enable Thrust File\": " << (enable_thrust_file ? "true" : "false") << ",\n"
        << "    \"Thrust File\": {\n"
        << "        \"Thrust at vacuum File Path\": \"" << csv_abs_path << "\"\n"
        << "    },\n"
        << "    \"Constant Thrust\": {\n"
        << "        \"Thrust at vacuum [N]\": 5780.0,\n"
        << "        \"Propellant Mass Flow Rate [kg/s]\": 3.0,\n"
        << "        \"Burn Duration [sec]\": 13.7\n"
        << "    },\n"
        << "    \"Enable Engine Miss Alignment\": " << (enable_misalign ? "true" : "false") << ",\n"
        << "    \"Engine Miss-Alignment\": {\n"
        << "        \"y-Axis Angle [deg]\": 2.0,\n"
        << "        \"z-Axis Angle [deg]\": -3.0\n"
        << "    }\n"
        << "}\n";
    ofs.close();
}

}  // namespace

// --- Branch 3: constant thrust, no misalignment -----------------------------
TEST(EngineFactory, ConstantThrustNoMisalignment) {
    WriteEngineJson(kJsonPath, /*thrust_file=*/false, /*misalign=*/false, kCsvPath);

    EngineFactory factory;
    Engine e = factory.Create(kJsonPath);

    EXPECT_DOUBLE_EQ(e.burn_duration, 13.7);  // "Burn Duration [sec]"
    // total_impulse = thrust * burn_duration = 5780.0 * 13.7
    EXPECT_DOUBLE_EQ(e.total_impulse, 5780.0 * 13.7);

    // No misalignment -> gimbal angles stay zero after Update.
    e.Update(1.0, 0.0, 5.0);
    EXPECT_DOUBLE_EQ(e.gimbal_angle_y_axis, 0.0);
    EXPECT_DOUBLE_EQ(e.gimbal_angle_z_axis, 0.0);
    // Vacuum thrust at p=0 equals the constant thrust value.
    EXPECT_DOUBLE_EQ(e.thrust, 5780.0);

    std::remove(kJsonPath);
}

// --- Branch 4: constant thrust, with misalignment ---------------------------
TEST(EngineFactory, ConstantThrustWithMisalignment) {
    WriteEngineJson(kJsonPath, /*thrust_file=*/false, /*misalign=*/true, kCsvPath);

    EngineFactory factory;
    Engine e = factory.Create(kJsonPath);

    EXPECT_DOUBLE_EQ(e.burn_duration, 13.7);
    EXPECT_DOUBLE_EQ(e.total_impulse, 5780.0 * 13.7);  // thrust * burn_duration

    // Misalignment becomes the gimbal angle after Update (deg -> rad using the
    // factory's truncated pi literal). y = 2 deg, z = -3 deg.
    e.Update(1.0, 0.0, 5.0);
    EXPECT_DOUBLE_EQ(e.gimbal_angle_y_axis, 2.0 / 180.0 * kPiTrunc);
    EXPECT_DOUBLE_EQ(e.gimbal_angle_z_axis, -3.0 / 180.0 * kPiTrunc);

    std::remove(kJsonPath);
}

// --- Branch 1: thrust file, no misalignment ---------------------------------
TEST(EngineFactory, ThrustFileNoMisalignment) {
    WriteThrustCsv(kCsvPath);
    WriteEngineJson(kJsonPath, /*thrust_file=*/true, /*misalign=*/false, kCsvPath);

    EngineFactory factory;
    Engine e = factory.Create(kJsonPath);

    // burn_duration = last time value in CSV = 10.
    EXPECT_DOUBLE_EQ(e.burn_duration, 10.0);
    // total_impulse = mean(thrust) * burn_duration = mean(1000,1000) * 10.
    EXPECT_DOUBLE_EQ(e.total_impulse, 1000.0 * 10.0);

    e.Update(5.0, 0.0, 5.0);  // within burn, p=0
    EXPECT_TRUE(e.burning);
    EXPECT_DOUBLE_EQ(e.thrust, 1000.0);   // interpolated constant thrust at p=0
    EXPECT_DOUBLE_EQ(e.gimbal_angle_y_axis, 0.0);
    EXPECT_DOUBLE_EQ(e.gimbal_angle_z_axis, 0.0);

    std::remove(kJsonPath);
    std::remove(kCsvPath);
}

// --- Branch 2: thrust file, with misalignment -------------------------------
TEST(EngineFactory, ThrustFileWithMisalignment) {
    WriteThrustCsv(kCsvPath);
    WriteEngineJson(kJsonPath, /*thrust_file=*/true, /*misalign=*/true, kCsvPath);

    EngineFactory factory;
    Engine e = factory.Create(kJsonPath);

    EXPECT_DOUBLE_EQ(e.burn_duration, 10.0);            // CSV last time
    EXPECT_DOUBLE_EQ(e.total_impulse, 1000.0 * 10.0);   // mean thrust * duration

    e.Update(5.0, 0.0, 5.0);
    EXPECT_TRUE(e.burning);
    // y = 2 deg, z = -3 deg, converted with the factory's truncated pi.
    EXPECT_DOUBLE_EQ(e.gimbal_angle_y_axis, 2.0 / 180.0 * kPiTrunc);
    EXPECT_DOUBLE_EQ(e.gimbal_angle_z_axis, -3.0 / 180.0 * kPiTrunc);

    std::remove(kJsonPath);
    std::remove(kCsvPath);
}

// Reference the computed exit area so the derivation is documented/used even
// though Engine does not expose area_exit publicly. This keeps the value under
// review (area_exit = 0.25 * pi * (D/1e3)^2) without a meaningless assertion on
// private state.
TEST(EngineFactory, ExitAreaDerivation_Characterization) {
    // 0.25 * pi * 0.1^2 = pi * 0.0025 ~= 0.007853981633974483
    EXPECT_NEAR(kAreaExit, 0.25 * kPi * 0.01, 1e-15);
}
