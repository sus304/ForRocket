// ******************************************************
// Unit tests for src/factory/rocket_stage_factory.cpp
//   RocketStageFactory::Create(stage_number, rocket_json, engine_json, soe_json)
//
// TARGET: >=80% branch (C1) on rocket_stage_factory.cpp.
//
// Strategy
// --------
// Create() takes FILE PATHS and the test CWD is the build dir, so every path
// passed in is an ABSOLUTE path under /tmp. To stay self-contained (no CSV
// dependencies, no reliance on CWD=examples) we synthesize minimal rocket and
// engine JSON that use ONLY constant parameters (constant-thrust engine,
// all-constant aero) so RocketFactory never touches a CSV file.
//
// Branch coverage of the SOE "Enable *" flags is achieved with two SOE
// variants exercising BOTH sides of every gate:
//   (1) AllDisabled  : every "Enable *" = false,
//                      "Enable Auto Terminate SubOrbital Flight" = false
//                      (-> reads "Flight End Time [s]"),
//                      no Solver Tolerance keys (-> contains() == false).
//   (2) AllEnabled   : every "Enable *" = true (launcher, cutoff, separation,
//                      despin, fairing, parachute, secondary parachute),
//                      "Enable Auto Terminate SubOrbital Flight" = true
//                      (-> computes time_end from total_impulse),
//                      Solver Tolerance Abs/Rel present (-> contains() == true).
//
// CdS verification note: setCdSParachute() only pushes into the (private)
// CdS_parachute_src vector; the public CdS_parachute stays 0 until
// OpenParachute() pops a value. We therefore call OpenParachute() to read back
// the value(s) the factory queued.
// ******************************************************

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>     // std::remove
#include <fstream>
#include <string>

#include "factory/rocket_stage_factory.hpp"
#include "solver/rocket_stage.hpp"

namespace forrocket {
namespace {

// ---- absolute temp paths (test CWD is the build dir) -----------------------
const char* kRocketJson = "/tmp/forrocket_test_rsf_rocket.json";
const char* kEngineJson = "/tmp/forrocket_test_rsf_engine.json";
const char* kSoeDisabled = "/tmp/forrocket_test_rsf_soe_disabled.json";
const char* kSoeEnabled  = "/tmp/forrocket_test_rsf_soe_enabled.json";

// Constant-thrust engine. No CSV -> "Enable Thrust File" = false.
// Thrust at vacuum = 1000 N, Burn Duration = 10 s, so the (constant) engine's
// total_impulse = 1000 * 10 = 10000 N.s (Engine::getTotalImpulse for constant).
void WriteEngineJson(const std::string& path) {
    std::ofstream ofs(path);
    ofs << "{\n"
        << "    \"Nozzle Exit Diameter [mm]\": 100.0,\n"
        << "    \"Enable Thrust File\": false,\n"
        << "    \"Thrust File\": {\n"
        << "        \"Thrust at vacuum File Path\": \"unused.csv\"\n"
        << "    },\n"
        << "    \"Constant Thrust\": {\n"
        << "        \"Thrust at vacuum [N]\": 1000.0,\n"
        << "        \"Propellant Mass Flow Rate [kg/s]\": 3.0,\n"
        << "        \"Burn Duration [sec]\": 10.0\n"
        << "    },\n"
        << "    \"Enable Engine Miss Alignment\": false,\n"
        << "    \"Engine Miss-Alignment\": {\n"
        << "        \"y-Axis Angle [deg]\": 0.0,\n"
        << "        \"z-Axis Angle [deg]\": 0.0\n"
        << "    }\n"
        << "}\n";
    ofs.close();
}

// All-constant rocket. Every "Enable * File" = false and aero/CG/CP/MI are
// constants, so RocketFactory never opens a CSV.
void WriteRocketJson(const std::string& path) {
    std::ofstream ofs(path);
    ofs << "{\n"
        << "    \"Diameter [mm]\": 180.0,\n"
        << "    \"Length [mm]\": 3900.0,\n"
        << "    \"Mass\": { \"Inert [kg]\": 52.5, \"Propellant [kg]\": 41.0 },\n"
        << "    \"Enable Gas Jet\": false,\n"
        << "    \"Gas Jet\": { \"Rolling Moment [N.m]\": 5.0, \"Duration [s]\": 2.5 },\n"
        << "    \"Enable Program Attitude\": false,\n"
        << "    \"Program Attitude\": { \"Mode\": \"Angle\", \"Enable Yaw\": true, \"Enable Pitch\": true, \"Enable Roll\": true, \"File Path\": \"unused.csv\" },\n"
        << "    \"Enable X-C.G. File\": false,\n"
        << "    \"X-C.G. File\": { \"X-C.G. File Path\": \"unused.csv\" },\n"
        << "    \"Constant X-C.G.\": { \"Constant X-C.G. from BodyTail [mm]\": 1100.0 },\n"
        << "    \"C.G. Offset\": { \"y-C.G. Offset [mm]\": 0.0, \"z-C.G. Offset [mm]\": 0.0 },\n"
        << "    \"Enable M.I. File\": false,\n"
        << "    \"M.I. File\": { \"M.I. File Path\": \"unused.csv\" },\n"
        << "    \"Constant M.I.\": { \"Yaw Axis [kg-m2]\": 45.0, \"Pitch Axis [kg-m2]\": 45.0, \"Roll Axis [kg-m2]\": 0.5 },\n"
        << "    \"Enable Product of Inertia\": false,\n"
        << "    \"Constant Product of Inertia\": { \"Ixy [kg-m2]\": 0.0, \"Ixz [kg-m2]\": 0.0, \"Iyz [kg-m2]\": 0.0 },\n"
        << "    \"Enable Product of Inertia File\": false,\n"
        << "    \"Product of Inertia File\": { \"Ixy File Path\": \"unused.csv\", \"Ixz File Path\": \"unused.csv\", \"Iyz File Path\": \"unused.csv\" },\n"
        << "    \"Enable X-C.P. File\": false,\n"
        << "    \"X-C.P. File\": { \"X-C.P. File Path\": \"unused.csv\" },\n"
        << "    \"Constant X-C.P.\": { \"Constant X-C.P. from BodyTail [mm]\": 835.0 },\n"
        << "    \"X-ThrustLoadingPoint from BodyTail [mm]\": 300.0,\n"
        << "    \"y-ThrustLoadingPoint Offset [mm]\": 0.0,\n"
        << "    \"z-ThrustLoadingPoint Offset [mm]\": 0.0,\n"
        << "    \"Enable CA File\": false,\n"
        << "    \"CA File\": { \"CA File Path\": \"unused.csv\", \"BurnOut CA File Path\": \"unused.csv\" },\n"
        << "    \"Constant CA\": { \"Constant CA [-]\": 0.4, \"Constant BurnOut CA [-]\": 0.5 },\n"
        << "    \"Enable CNa File\": false,\n"
        << "    \"CNa File\": { \"CNa File Path\": \"unused.csv\" },\n"
        << "    \"Constant CNa\": { \"Constant CNa [1/rad]\": 10.0 },\n"
        << "    \"Fin Cant Angle [deg]\": 0.0,\n"
        << "    \"Enable Cld File\": false,\n"
        << "    \"Cld File\": { \"Cld File Path\": \"unused.csv\" },\n"
        << "    \"Constant Cld\": { \"Constant Cld [1/rad]\": 0.0 },\n"
        << "    \"Enable Clp File\": false,\n"
        << "    \"Clp File\": { \"Clp File Path\": \"unused.csv\" },\n"
        << "    \"Constant Clp\": { \"Constant Clp [-]\": 0.03 },\n"
        << "    \"Enable Cmq File\": false,\n"
        << "    \"Cmq File\": { \"Cmq File Path\": \"unused.csv\" },\n"
        << "    \"Constant Cmq\": { \"Constant Cmq [-]\": 7.0 },\n"
        << "    \"Enable Cnr File\": false,\n"
        << "    \"Cnr File\": { \"Cnr File Path\": \"unused.csv\" },\n"
        << "    \"Constant Cnr\": { \"Constant Cnr [-]\": 7.0 }\n"
        << "}\n";
    ofs.close();
}

// Variant (1): every Enable flag false, auto-terminate false, no tolerance keys.
void WriteSoeDisabled(const std::string& path) {
    std::ofstream ofs(path);
    ofs << "{\n"
        << "    \"Flight Start Time [s]\": 1.5,\n"
        << "    \"Engine Ignittion Time [s]\": 2.5,\n"
        << "    \"Enable Rail-Launcher Launch\": false,\n"
        << "    \"Rail Launcher\": { \"Length [m]\": 5.0 },\n"
        << "    \"Enable Engine Cutoff\": false,\n"
        << "    \"Cutoff\": { \"Cutoff Time [s]\": 8.0 },\n"
        << "    \"Enable Stage Separation\": false,\n"
        << "    \"Upper Stage\": { \"Stage Separation Time [s]\": 12.0, \"Upper Stage Mass [kg]\": 100.0 },\n"
        << "    \"Enable Despin Control\": false,\n"
        << "    \"Despin\": { \"Time [s]\": 20.0 },\n"
        << "    \"Enable Fairing Jettson\": false,\n"
        << "    \"Fairing\": { \"Jettson Time [s]\": 15.0, \"Mass [kg]\": 1.0 },\n"
        << "    \"Enable Parachute Open\": false,\n"
        << "    \"Parachute\": { \"Open Time [s]\": 30.0, \"Drag Factor Cd*S [m2]\": 0.3, \"Enable Forced Apogee Open\": false },\n"
        << "    \"Enable Secondary Parachute Open\": false,\n"
        << "    \"Secondary Parachute\": { \"Open Time [s]\": 60.0, \"Drag Factor Cd*S [m2]\": 1.3 },\n"
        << "    \"Flight End Time [s]\": 123.0,\n"
        << "    \"Time Step [s]\": 0.1,\n"
        << "    \"Enable Auto Terminate SubOrbital Flight\": false\n"
        << "}\n";
    ofs.close();
}

// Variant (2): every Enable flag true, auto-terminate true, tolerance keys set.
void WriteSoeEnabled(const std::string& path) {
    std::ofstream ofs(path);
    ofs << "{\n"
        << "    \"Flight Start Time [s]\": 0.0,\n"
        << "    \"Engine Ignittion Time [s]\": 0.0,\n"
        << "    \"Enable Rail-Launcher Launch\": true,\n"
        << "    \"Rail Launcher\": { \"Length [m]\": 7.0 },\n"
        << "    \"Enable Engine Cutoff\": true,\n"
        << "    \"Cutoff\": { \"Cutoff Time [s]\": 8.0 },\n"
        << "    \"Enable Stage Separation\": true,\n"
        << "    \"Upper Stage\": { \"Stage Separation Time [s]\": 12.0, \"Upper Stage Mass [kg]\": 33.0 },\n"
        << "    \"Enable Despin Control\": true,\n"
        << "    \"Despin\": { \"Time [s]\": 21.0 },\n"
        << "    \"Enable Fairing Jettson\": true,\n"
        << "    \"Fairing\": { \"Jettson Time [s]\": 15.0, \"Mass [kg]\": 2.0 },\n"
        << "    \"Enable Parachute Open\": true,\n"
        << "    \"Parachute\": { \"Open Time [s]\": 30.0, \"Drag Factor Cd*S [m2]\": 0.3, \"Enable Forced Apogee Open\": true },\n"
        << "    \"Enable Secondary Parachute Open\": true,\n"
        << "    \"Secondary Parachute\": { \"Open Time [s]\": 60.0, \"Drag Factor Cd*S [m2]\": 1.3 },\n"
        << "    \"Flight End Time [s]\": 123.0,\n"
        << "    \"Time Step [s]\": 0.2,\n"
        << "    \"Solver Tolerance Abs\": 1.0e-5,\n"
        << "    \"Solver Tolerance Rel\": 1.0e-4,\n"
        << "    \"Enable Auto Terminate SubOrbital Flight\": true\n"
        << "}\n";
    ofs.close();
}

// Fixture that lays down the self-contained rocket+engine JSON once per test and
// cleans every temp file (rocket, engine, both SOE variants) afterwards.
class RocketStageFactoryTest : public ::testing::Test {
 protected:
    void SetUp() override {
        WriteRocketJson(kRocketJson);
        WriteEngineJson(kEngineJson);
    }
    void TearDown() override {
        std::remove(kRocketJson);
        std::remove(kEngineJson);
        std::remove(kSoeDisabled);
        std::remove(kSoeEnabled);
    }
};

}  // namespace

// ---------------------------------------------------------------------------
// Variant (1): the FALSE side of every Enable gate + Flight-End-Time branch +
// contains()==false for both solver-tolerance optionals.
// ---------------------------------------------------------------------------
TEST_F(RocketStageFactoryTest, AllDisabledFalseBranches) {
    WriteSoeDisabled(kSoeDisabled);

    RocketStageFactory factory;
    RocketStage stage = factory.Create(1, kRocketJson, kEngineJson, kSoeDisabled);

    // stage_number passed straight through.
    EXPECT_EQ(stage.stage_number, 1);

    // time_start / time_ignittion read verbatim from the SOE JSON.
    EXPECT_DOUBLE_EQ(stage.time_start, 1.5);       // "Flight Start Time [s]"
    EXPECT_DOUBLE_EQ(stage.time_ignittion, 2.5);   // "Engine Ignittion Time [s]"

    // Every Enable flag took its FALSE branch.
    EXPECT_FALSE(stage.enable_launcher);
    EXPECT_FALSE(stage.enable_cutoff);
    EXPECT_FALSE(stage.enable_sepation);
    EXPECT_FALSE(stage.enable_despin);
    EXPECT_FALSE(stage.enable_fairing_jettson);
    EXPECT_FALSE(stage.enable_parachute_open);
    EXPECT_FALSE(stage.exist_second_parachute);

    // Auto-terminate FALSE branch -> time_end == "Flight End Time [s]".
    EXPECT_DOUBLE_EQ(stage.time_end, 123.0);

    // time_step read verbatim.
    EXPECT_DOUBLE_EQ(stage.time_step, 0.1);

    // No "Solver Tolerance *" keys -> contains() == false on both -> the
    // RocketStage constructor defaults (1e-6 / 1e-6) survive untouched.
    EXPECT_DOUBLE_EQ(stage.eps_abs, 1.0e-6);  // RocketStage ctor default
    EXPECT_DOUBLE_EQ(stage.eps_rel, 1.0e-6);  // RocketStage ctor default
}

// ---------------------------------------------------------------------------
// Variant (2): the TRUE side of every Enable gate + auto-terminate branch +
// contains()==true for both solver-tolerance optionals.
// ---------------------------------------------------------------------------
TEST_F(RocketStageFactoryTest, AllEnabledTrueBranches) {
    WriteSoeEnabled(kSoeEnabled);

    RocketStageFactory factory;
    RocketStage stage = factory.Create(2, kRocketJson, kEngineJson, kSoeEnabled);

    EXPECT_EQ(stage.stage_number, 2);
    EXPECT_DOUBLE_EQ(stage.time_start, 0.0);
    EXPECT_DOUBLE_EQ(stage.time_ignittion, 0.0);

    // --- Rail-Launcher Launch TRUE branch ---
    EXPECT_TRUE(stage.enable_launcher);
    EXPECT_DOUBLE_EQ(stage.length_launcher_rail, 7.0);  // "Rail Launcher"/"Length [m]"

    // --- Engine Cutoff TRUE branch ---
    EXPECT_TRUE(stage.enable_cutoff);
    EXPECT_DOUBLE_EQ(stage.time_cutoff, 8.0);            // "Cutoff"/"Cutoff Time [s]"
    // Cutoff also overwrites the engine burn_duration with time_cutoff.
    EXPECT_DOUBLE_EQ(stage.rocket.engine.burn_duration, 8.0);

    // --- Stage Separation TRUE branch ---
    EXPECT_TRUE(stage.enable_sepation);
    EXPECT_DOUBLE_EQ(stage.time_separation, 12.0);       // "Stage Separation Time [s]"
    EXPECT_DOUBLE_EQ(stage.mass_upper_stage, 33.0);      // "Upper Stage Mass [kg]"

    // --- Despin Control TRUE branch ---
    EXPECT_TRUE(stage.enable_despin);
    EXPECT_DOUBLE_EQ(stage.time_despin, 21.0);           // "Despin"/"Time [s]"

    // --- Fairing Jettson TRUE branch ---
    EXPECT_TRUE(stage.enable_fairing_jettson);
    EXPECT_DOUBLE_EQ(stage.time_jettson_fairing, 15.0);  // "Fairing"/"Jettson Time [s]"
    EXPECT_DOUBLE_EQ(stage.mass_fairing, 2.0);           // "Fairing"/"Mass [kg]"

    // --- Parachute Open TRUE branch (incl. forced-apogee-open sub-flag) ---
    EXPECT_TRUE(stage.enable_parachute_open);
    EXPECT_TRUE(stage.enable_apogee_parachute_open);     // "Enable Forced Apogee Open"
    EXPECT_DOUBLE_EQ(stage.time_open_parachute, 30.0);   // "Parachute"/"Open Time [s]"

    // --- Secondary Parachute Open TRUE branch ---
    EXPECT_TRUE(stage.exist_second_parachute);
    EXPECT_DOUBLE_EQ(stage.time_open_second_parachute, 60.0);

    // Cd*S verification: the factory called setCdSParachute() twice (primary
    // then secondary), queuing [0.3, 1.3] into the private CdS_parachute_src.
    // OpenParachute() pops them in order onto the public CdS_parachute (which
    // accumulates). First pop -> primary 0.3, second pop -> +secondary 1.3.
    EXPECT_DOUBLE_EQ(stage.rocket.CdS_parachute, 0.0);   // nothing opened yet
    stage.rocket.OpenParachute();
    EXPECT_DOUBLE_EQ(stage.rocket.CdS_parachute, 0.3);   // primary CdS*S
    stage.rocket.OpenParachute();
    EXPECT_DOUBLE_EQ(stage.rocket.CdS_parachute, 0.3 + 1.3);  // + secondary

    // --- Auto-terminate TRUE branch ---
    // time_end = 5.0 * |60*ln(total_impulse) - 410|, total_impulse = 1000*10 =
    // 10000 (constant-thrust engine). Computed: ~713.1021115928553 s.
    const double expected_total_impulse = 1000.0 * 10.0;  // thrust * burn_dur
    const double expected_time_end =
            5.0 * std::abs(60.0 * std::log(expected_total_impulse) - 410.0);
    EXPECT_NEAR(stage.time_end, expected_time_end, 1e-9);
    EXPECT_NEAR(stage.time_end, 713.1021115928553, 1e-6);  // sanity literal

    // time_step read verbatim.
    EXPECT_DOUBLE_EQ(stage.time_step, 0.2);

    // --- contains()==true branch for both solver tolerances ---
    EXPECT_DOUBLE_EQ(stage.eps_abs, 1.0e-5);  // "Solver Tolerance Abs"
    EXPECT_DOUBLE_EQ(stage.eps_rel, 1.0e-4);  // "Solver Tolerance Rel"
}

}  // namespace forrocket
