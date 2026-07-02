// ******************************************************
// Unit tests for src/solver/trajectory_solver.cpp  (THIN / smoke only)
//
// Scope rationale
// ---------------
// TrajectorySolver has three members:
//   - TrajectorySolver(json)  : parses the solver config, builds every stage via
//                               RocketStageFactory, sets up the FlightDataRecorder,
//                               the master clock, and initializes the first
//                               stage's position/velocity/attitude at the launch
//                               site. No integration is performed here.
//   - Solve()                 : runs the FULL adaptive-stepper trajectory for
//                               every stage (boost odeint). This is heavy and is
//                               already exercised end-to-end by the Python E2E
//                               suite (tests/simulation_test.py).
//   - DumpResult()            : writes per-stage CSV via FlightDataRecorder.
//
// Solve() is integration-level: it cannot be meaningfully unit-tested without
// running a whole flight, so it is INTENTIONALLY LEFT to the E2E suite. The only
// piece that is cheaply testable in isolation is the constructor, so this file
// is a focused construction smoke test:
//   * config parsing (Model ID, Number of Stage),
//   * stage construction wiring (factory output reachable via stage_vector),
//   * the "Enable Wind" == false branch (no wind CSV needed),
//   * first-stage kinematic initialization (ECI position ~ Earth radius).
//
// Fixture strategy mirrors test_rocket_stage_factory.cpp: a fully self-contained
// config chain written under /tmp using ABSOLUTE paths only (test CWD is the
// build dir), with constant-only rocket+engine JSON so no CSV is ever opened.
// ******************************************************

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>     // std::remove
#include <fstream>
#include <string>

#include "solver/trajectory_solver.hpp"

namespace forrocket {
namespace {

// --- absolute temp paths (the constructor stores & later opens these) -------
const char* kRocketJson = "/tmp/forrocket_test_ts_rocket.json";
const char* kEngineJson = "/tmp/forrocket_test_ts_engine.json";
const char* kSoeJson    = "/tmp/forrocket_test_ts_soe.json";
const char* kStageList  = "/tmp/forrocket_test_ts_stage1_list.json";
const char* kSolverJson = "/tmp/forrocket_test_ts_solver.json";

void WriteEngineJson(const std::string& path) {
    std::ofstream ofs(path);
    ofs << "{\n"
        << "    \"Nozzle Exit Diameter [mm]\": 100.0,\n"
        << "    \"Enable Thrust File\": false,\n"
        << "    \"Thrust File\": { \"Thrust at vacuum File Path\": \"unused.csv\" },\n"
        << "    \"Constant Thrust\": { \"Thrust at vacuum [N]\": 1000.0, \"Propellant Mass Flow Rate [kg/s]\": 3.0, \"Burn Duration [sec]\": 10.0 },\n"
        << "    \"Enable Engine Miss Alignment\": false,\n"
        << "    \"Engine Miss-Alignment\": { \"y-Axis Angle [deg]\": 0.0, \"z-Axis Angle [deg]\": 0.0 }\n"
        << "}\n";
    ofs.close();
}

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

void WriteSoeJson(const std::string& path) {
    std::ofstream ofs(path);
    ofs << "{\n"
        << "    \"Flight Start Time [s]\": 0.0,\n"
        << "    \"Engine Ignittion Time [s]\": 0.0,\n"
        << "    \"Enable Rail-Launcher Launch\": true,\n"
        << "    \"Rail Launcher\": { \"Length [m]\": 5.0 },\n"
        << "    \"Enable Engine Cutoff\": false,\n"
        << "    \"Cutoff\": { \"Cutoff Time [s]\": 0.0 },\n"
        << "    \"Enable Stage Separation\": false,\n"
        << "    \"Upper Stage\": { \"Stage Separation Time [s]\": 0.0, \"Upper Stage Mass [kg]\": 100.0 },\n"
        << "    \"Enable Despin Control\": false,\n"
        << "    \"Despin\": { \"Time [s]\": 20.0 },\n"
        << "    \"Enable Fairing Jettson\": false,\n"
        << "    \"Fairing\": { \"Jettson Time [s]\": 0.0, \"Mass [kg]\": 1.0 },\n"
        << "    \"Enable Parachute Open\": false,\n"
        << "    \"Parachute\": { \"Open Time [s]\": 30.0, \"Drag Factor Cd*S [m2]\": 0.3, \"Enable Forced Apogee Open\": false },\n"
        << "    \"Enable Secondary Parachute Open\": false,\n"
        << "    \"Secondary Parachute\": { \"Open Time [s]\": 60.0, \"Drag Factor Cd*S [m2]\": 1.3 },\n"
        << "    \"Flight End Time [s]\": 100.0,\n"
        << "    \"Time Step [s]\": 0.1,\n"
        << "    \"Enable Auto Terminate SubOrbital Flight\": false\n"
        << "}\n";
    ofs.close();
}

// Stage-config-list references the rocket/engine/SOE by ABSOLUTE path so the
// factory can open them regardless of CWD.
void WriteStageList(const std::string& path) {
    std::ofstream ofs(path);
    ofs << "{\n"
        << "    \"Rocket Configuration File Path\": \"" << kRocketJson << "\",\n"
        << "    \"Engine Configuration File Path\": \"" << kEngineJson << "\",\n"
        << "    \"Sequence of Event File Path\": \"" << kSoeJson << "\"\n"
        << "}\n";
    ofs.close();
}

// Solver config: single stage, wind DISABLED (exercises EnvironmentWind(false)
// branch -> no wind CSV needed), launch site identical to the bundled sample.
// gravity_model: "" omits the optional "Gravity Model" key entirely.
void WriteSolverJson(const std::string& path, const std::string& gravity_model = "") {
    std::ofstream ofs(path);
    ofs << "{\n"
        << "    \"Model ID\": \"smoke\",\n";
    if (!gravity_model.empty()) {
        ofs << "    \"Gravity Model\": \"" << gravity_model << "\",\n";
    }
    ofs << "    \"Launch DateTime\": \"2020/08/23 9:00:00.0\",\n"
        << "    \"Launch Condition\": {\n"
        << "        \"Latitude [deg]\": 40.242865,\n"
        << "        \"Longitude [deg]\": 140.01045,\n"
        << "        \"Height for WGS84 [m]\": 20.0,\n"
        << "        \"Azimuth [deg]\": 270.0,\n"
        << "        \"Elevation [deg]\": 85.0,\n"
        << "        \"North Velocity [m/s]\": 0.0,\n"
        << "        \"East Velocity [m/s]\": 0.0,\n"
        << "        \"Down Velocity [m/s]\": 0.0,\n"
        << "        \"Yaw Angular Velocity [deg/s]\": 0.0,\n"
        << "        \"Pitch Angular Velocity [deg/s]\": 0.0,\n"
        << "        \"Roll Angular Velocity [deg/s]\": 0.0\n"
        << "    },\n"
        << "    \"Wind Condition\": { \"Enable Wind\": false, \"Wind File Path\": \"unused.csv\" },\n"
        << "    \"Number of Stage\": 1,\n"
        << "    \"Stage1 Config File List\": \"" << kStageList << "\"\n"
        << "}\n";
    ofs.close();
}

class TrajectorySolverTest : public ::testing::Test {
 protected:
    void SetUp() override {
        WriteRocketJson(kRocketJson);
        WriteEngineJson(kEngineJson);
        WriteSoeJson(kSoeJson);
        WriteStageList(kStageList);
        WriteSolverJson(kSolverJson);
    }
    void TearDown() override {
        std::remove(kRocketJson);
        std::remove(kEngineJson);
        std::remove(kSoeJson);
        std::remove(kStageList);
        std::remove(kSolverJson);
    }
};

}  // namespace

// ---------------------------------------------------------------------------
// Constructor smoke: config parsing + stage wiring + wind-disabled branch +
// first-stage kinematic initialization. No Solve() is run.
// ---------------------------------------------------------------------------
TEST_F(TrajectorySolverTest, ConstructSingleStage_Smoke) {
    TrajectorySolver solver(kSolverJson);

    // --- config parsing ---
    EXPECT_EQ(solver.model_id, "smoke");   // "Model ID"
    EXPECT_EQ(solver.number_stage, 1);     // "Number of Stage"

    // One stage built and pushed into the vector.
    ASSERT_EQ(solver.stage_vector.size(), 1u);

    // --- stage wiring: factory output is reachable via stage_vector[0] ---
    RocketStage& s0 = solver.stage_vector[0];
    EXPECT_EQ(s0.stage_number, 1);                 // 1-based loop index
    EXPECT_DOUBLE_EQ(s0.time_start, 0.0);          // SOE "Flight Start Time [s]"
    EXPECT_TRUE(s0.enable_launcher);               // SOE "Enable Rail-Launcher Launch"
    EXPECT_DOUBLE_EQ(s0.length_launcher_rail, 5.0);// SOE "Rail Launcher"/"Length [m]"
    EXPECT_FALSE(s0.enable_sepation);              // single stage -> no separation
    EXPECT_DOUBLE_EQ(s0.time_end, 100.0);          // SOE "Flight End Time [s]" (auto-term off)

    // --- first-stage kinematic initialization ---
    // position.Initialize() places the rocket on the WGS84 ellipsoid at the
    // launch site, so |ECI| is on the order of Earth's radius (~6.37e6 m), not
    // the all-zero default. Use a wide band so the check is robust to the exact
    // datetime/sidereal rotation (we only assert "was initialized to a sane,
    // non-zero geocentric radius").
    double r_eci = s0.rocket.position.ECI.norm();
    EXPECT_GT(r_eci, 6.0e6);   // _Characterization: > min plausible geocentric radius
    EXPECT_LT(r_eci, 6.6e6);   // _Characterization: < max plausible geocentric radius

    // Attitude quaternion was initialized to a unit quaternion (norm == 1).
    EXPECT_NEAR(s0.rocket.attitude.quaternion.norm(), 1.0, 1e-9);

    // --- master clock seeded from "Launch DateTime" (countup starts at 0) ---
    EXPECT_DOUBLE_EQ(solver.master_clock.countup_time, 0.0);
}

// ---------------------------------------------------------------------------
// "Gravity Model" (optional solver-config key, added with the pointmass-j2
// gravity option): omitted / "legacy" -> flag false; "pointmass-j2" -> flag
// true on every stage; unknown value -> exit(EXIT_FAILURE) with a message.
// ---------------------------------------------------------------------------
TEST_F(TrajectorySolverTest, GravityModelOmittedDefaultsToLegacy) {
    TrajectorySolver solver(kSolverJson);  // fixture writes no "Gravity Model" key
    EXPECT_FALSE(solver.stage_vector[0].rocket.gravity_model_j2);
}

TEST_F(TrajectorySolverTest, GravityModelLegacyKeepsFlagFalse) {
    WriteSolverJson(kSolverJson, "legacy");
    TrajectorySolver solver(kSolverJson);
    EXPECT_FALSE(solver.stage_vector[0].rocket.gravity_model_j2);
}

TEST_F(TrajectorySolverTest, GravityModelPointMassJ2SetsFlag) {
    WriteSolverJson(kSolverJson, "pointmass-j2");
    TrajectorySolver solver(kSolverJson);
    EXPECT_TRUE(solver.stage_vector[0].rocket.gravity_model_j2);
}

TEST_F(TrajectorySolverTest, GravityModelUnknownValueExits) {
    WriteSolverJson(kSolverJson, "j4-full");
    EXPECT_EXIT({ TrajectorySolver solver(kSolverJson); },
                ::testing::ExitedWithCode(EXIT_FAILURE),
                "Undefined Gravity Model");
}

}  // namespace forrocket
