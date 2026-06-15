// ******************************************************
// Unit tests for src/factory/rocket_factory.cpp
//
// Goal: >= 80% BRANCH (C1) coverage of RocketFactory::Create. Create() reads a
// rocket-config JSON (and an engine-config JSON) and, for every parameter,
// chooses between a CSV-log branch (`if (jc.getBool("Enable X File"))`) and a
// constant branch (`else`). It also has contains()-gated optional blocks:
//   - "C.G. Offset" vs legacy nested keys under "Constant X-C.G."
//   - "Enable Product of Inertia File" / "Enable Product of Inertia" / neither
//   - "y/z-ThrustLoadingPoint Offset" present vs absent
//   - "Enable Gas Jet", "Enable Program Attitude" (Mode "Rate" vs "Angle")
//
// Strategy: Create() takes FILE PATHS, and the test CWD is the build dir, so we
// materialise temp JSON + CSV files under /tmp with ABSOLUTE paths in SetUp and
// remove them in TearDown. Three rocket-JSON variants exercise both sides of
// every flag:
//   (1) ALL-CONSTANT  : every Enable*=false, C.G. Offset block present.
//   (2) ALL-FILE      : every Enable*=true, Program Attitude Mode "Angle",
//                       Product of Inertia *File*, C.G. Offset block OMITTED
//                       (legacy y/z under "Constant X-C.G.").
//   (3) RATE+POI-CONST: Program Attitude Mode "Rate", constant Product of
//                       Inertia ("Enable Product of Inertia"=true).
//
// All assertions derive from the JSON/CSV inputs written below; each hardcoded
// expected value carries a derivation comment.
// ******************************************************

#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "Eigen/Dense"

#include "factory/rocket_factory.hpp"
#include "rocket/rocket.hpp"

namespace forrocket {
namespace {

const double kPi = 3.14159265358979323846;

// ---- temp file paths (absolute, under /tmp) -----------------------------
const char* kEngineJson    = "/tmp/forrocket_test_engine.json";
const char* kRocketConst   = "/tmp/forrocket_test_rocket_const.json";
const char* kRocketFile    = "/tmp/forrocket_test_rocket_file.json";
const char* kRocketRate    = "/tmp/forrocket_test_rocket_rate.json";

// 2-column CSV (mach/time, value); header + 2 data rows so interpolation has a
// real table. Querying exactly at a node returns that node's value.
const char* kCsvXcg        = "/tmp/forrocket_test_xcg.csv";        // 0->1.1 m worth
const char* kCsvXcp        = "/tmp/forrocket_test_xcp.csv";
const char* kCsvCA         = "/tmp/forrocket_test_ca.csv";
const char* kCsvCAburn     = "/tmp/forrocket_test_ca_burn.csv";
const char* kCsvCNa        = "/tmp/forrocket_test_cna.csv";
const char* kCsvCld        = "/tmp/forrocket_test_cld.csv";
const char* kCsvClp        = "/tmp/forrocket_test_clp.csv";
const char* kCsvCmq        = "/tmp/forrocket_test_cmq.csv";
const char* kCsvCnr        = "/tmp/forrocket_test_cnr.csv";
const char* kCsvIxy        = "/tmp/forrocket_test_ixy.csv";
const char* kCsvIxz        = "/tmp/forrocket_test_ixz.csv";
const char* kCsvIyz        = "/tmp/forrocket_test_iyz.csv";
// 4-column CSV (time, yaw/Iyy, pitch/Ipp, roll/Irr).
const char* kCsvMOI        = "/tmp/forrocket_test_moi.csv";
const char* kCsvAtt        = "/tmp/forrocket_test_att.csv";

void WriteFile(const char* path, const std::string& contents) {
    std::ofstream ofs(path);
    ofs << contents;
    ofs.close();
}

class RocketFactoryTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // --- Engine config: constant thrust, no thrust file, no misalignment.
        // Mirrors examples/sample_param_engine.json but self-contained.
        WriteFile(kEngineJson,
            "{\n"
            "  \"Nozzle Exit Diameter [mm]\": 100.0,\n"
            "  \"Enable Thrust File\": false,\n"
            "  \"Thrust File\": { \"Thrust at vacuum File Path\": \"x.csv\" },\n"
            "  \"Constant Thrust\": {\n"
            "    \"Thrust at vacuum [N]\": 5780.0,\n"
            "    \"Propellant Mass Flow Rate [kg/s]\": 3.0,\n"
            "    \"Burn Duration [sec]\": 13.7\n"
            "  },\n"
            "  \"Enable Engine Miss Alignment\": false,\n"
            "  \"Engine Miss-Alignment\": { \"y-Axis Angle [deg]\": 0.0, \"z-Axis Angle [deg]\": 0.0 }\n"
            "}\n");

        // --- 2-column CSVs. First column is mach (X-C.P./CA/CNa/...) or time
        //     (X-C.G./Ixy...). Node 0 -> first value, node 10 -> second value.
        WriteFile(kCsvXcg,    "time,xcg\n0,1.234\n10,1.5\n");      // m at t=0 -> 1.234
        WriteFile(kCsvXcp,    "mach,xcp\n0,0.876\n10,0.9\n");      // m at mach=0 -> 0.876
        WriteFile(kCsvCA,     "mach,ca\n0,0.41\n10,0.5\n");        // burning CA at mach0
        WriteFile(kCsvCAburn, "mach,ca\n0,0.55\n10,0.6\n");        // burnout CA at mach0
        WriteFile(kCsvCNa,    "mach,cna\n0,11.0\n10,12.0\n");      // 1/rad
        WriteFile(kCsvCld,    "mach,cld\n0,0.02\n10,0.03\n");
        WriteFile(kCsvClp,    "mach,clp\n0,-0.04\n10,-0.05\n");    // already <=0
        WriteFile(kCsvCmq,    "mach,cmq\n0,-6.0\n10,-7.0\n");
        WriteFile(kCsvCnr,    "mach,cnr\n0,-6.5\n10,-7.5\n");
        WriteFile(kCsvIxy,    "time,ixy\n0,0.07\n10,0.08\n");      // kg-m2 at t=0
        WriteFile(kCsvIxz,    "time,ixz\n0,0.09\n10,0.10\n");
        WriteFile(kCsvIyz,    "time,iyz\n0,0.11\n10,0.12\n");

        // --- 4-column CSVs.
        // M.I. file: time, yaw(Iyy), pitch(Ipp), roll(Irr). At t=0 -> 40,41,2.
        WriteFile(kCsvMOI,    "time,yaw,pitch,roll\n0,40.0,41.0,2.0\n10,30.0,31.0,1.5\n");
        // Program attitude file: time, yaw, pitch, roll (deg or deg/s).
        // 90 deg at all nodes -> after *pi/180 conversion -> pi/2 rad (or rad/s).
        WriteFile(kCsvAtt,    "time,yaw,pitch,roll\n0,90.0,90.0,90.0\n10,90.0,90.0,90.0\n");

        // --- Variant (1): ALL CONSTANT. Every Enable*=false, C.G. Offset block
        //     present (contains("C.G. Offset")==true), y/z thrust offsets present.
        WriteFile(kRocketConst,
            "{\n"
            "  \"Diameter [mm]\": 180.0,\n"
            "  \"Length [mm]\": 3900.0,\n"
            "  \"Mass\": { \"Inert [kg]\": 52.5, \"Propellant [kg]\": 41.0 },\n"
            "  \"Enable Gas Jet\": false,\n"
            "  \"Gas Jet\": { \"Rolling Moment [N.m]\": 5.0, \"Duration [s]\": 2.5 },\n"
            "  \"Enable Program Attitude\": false,\n"
            "  \"Program Attitude\": { \"Mode\": \"Angle\", \"Enable Yaw\": true,"
            " \"Enable Pitch\": true, \"Enable Roll\": true, \"File Path\": \"a.csv\" },\n"
            "  \"Enable X-C.G. File\": false,\n"
            "  \"X-C.G. File\": { \"X-C.G. File Path\": \"x.csv\" },\n"
            "  \"Constant X-C.G.\": { \"Constant X-C.G. from BodyTail [mm]\": 1100.0 },\n"
            "  \"C.G. Offset\": { \"y-C.G. Offset [mm]\": 12.0, \"z-C.G. Offset [mm]\": -8.0 },\n"
            "  \"Enable M.I. File\": false,\n"
            "  \"M.I. File\": { \"M.I. File Path\": \"x.csv\" },\n"
            "  \"Constant M.I.\": { \"Yaw Axis [kg-m2]\": 45.0, \"Pitch Axis [kg-m2]\": 46.0, \"Roll Axis [kg-m2]\": 0.5 },\n"
            "  \"Enable Product of Inertia\": false,\n"
            "  \"Constant Product of Inertia\": { \"Ixy [kg-m2]\": 0.0, \"Ixz [kg-m2]\": 0.0, \"Iyz [kg-m2]\": 0.0 },\n"
            "  \"Enable Product of Inertia File\": false,\n"
            "  \"Product of Inertia File\": { \"Ixy File Path\": \"x.csv\", \"Ixz File Path\": \"x.csv\", \"Iyz File Path\": \"x.csv\" },\n"
            "  \"Enable X-C.P. File\": false,\n"
            "  \"X-C.P. File\": { \"X-C.P. File Path\": \"x.csv\" },\n"
            "  \"Constant X-C.P.\": { \"Constant X-C.P. from BodyTail [mm]\": 835.0 },\n"
            "  \"X-ThrustLoadingPoint from BodyTail [mm]\": 300.0,\n"
            "  \"y-ThrustLoadingPoint Offset [mm]\": 7.0,\n"
            "  \"z-ThrustLoadingPoint Offset [mm]\": -3.0,\n"
            "  \"Enable CA File\": false,\n"
            "  \"CA File\": { \"CA File Path\": \"x.csv\", \"BurnOut CA File Path\": \"x.csv\" },\n"
            "  \"Constant CA\": { \"Constant CA [-]\": 0.40, \"Constant BurnOut CA [-]\": 0.50 },\n"
            "  \"Enable CNa File\": false,\n"
            "  \"CNa File\": { \"CNa File Path\": \"x.csv\" },\n"
            "  \"Constant CNa\": { \"Constant CNa [1/rad]\": 10.0 },\n"
            "  \"Fin Cant Angle [deg]\": 0.0,\n"
            "  \"Enable Cld File\": false,\n"
            "  \"Cld File\": { \"Cld File Path\": \"x.csv\" },\n"
            "  \"Constant Cld\": { \"Constant Cld [1/rad]\": 0.0 },\n"
            "  \"Enable Clp File\": false,\n"
            "  \"Clp File\": { \"Clp File Path\": \"x.csv\" },\n"
            "  \"Constant Clp\": { \"Constant Clp [-]\": 0.03 },\n"
            "  \"Enable Cmq File\": false,\n"
            "  \"Cmq File\": { \"Cmq File Path\": \"x.csv\" },\n"
            "  \"Constant Cmq\": { \"Constant Cmq [-]\": 7.0 },\n"
            "  \"Enable Cnr File\": false,\n"
            "  \"Cnr File\": { \"Cnr File Path\": \"x.csv\" },\n"
            "  \"Constant Cnr\": { \"Constant Cnr [-]\": 7.0 }\n"
            "}\n");

        // --- Variant (2): ALL FILE. Every Enable*=true, Gas Jet on, Program
        //     Attitude on (Mode "Angle"), Product of Inertia *File* on.
        //     NO "C.G. Offset" block -> exercises the legacy else-branch that
        //     reads y/z C.G. Offset from under "Constant X-C.G.".
        WriteFile(kRocketFile,
            std::string("{\n"
            "  \"Diameter [mm]\": 200.0,\n"
            "  \"Length [mm]\": 4000.0,\n"
            "  \"Mass\": { \"Inert [kg]\": 60.0, \"Propellant [kg]\": 30.0 },\n"
            "  \"Enable Gas Jet\": true,\n"
            "  \"Gas Jet\": { \"Rolling Moment [N.m]\": 4.5, \"Duration [s]\": 3.0 },\n"
            "  \"Enable Program Attitude\": true,\n"
            "  \"Program Attitude\": { \"Mode\": \"Angle\", \"Enable Yaw\": false,"
            " \"Enable Pitch\": true, \"Enable Roll\": false, \"File Path\": \"") + kCsvAtt + "\" },\n"
            "  \"Enable X-C.G. File\": true,\n"
            "  \"X-C.G. File\": { \"X-C.G. File Path\": \"" + kCsvXcg + "\" },\n"
            "  \"Constant X-C.G.\": { \"Constant X-C.G. from BodyTail [mm]\": 1100.0,"
            " \"y-C.G. Offset [mm]\": 21.0, \"z-C.G. Offset [mm]\": -5.0 },\n"
            "  \"Enable M.I. File\": true,\n"
            "  \"M.I. File\": { \"M.I. File Path\": \"" + kCsvMOI + "\" },\n"
            "  \"Constant M.I.\": { \"Yaw Axis [kg-m2]\": 45.0, \"Pitch Axis [kg-m2]\": 45.0, \"Roll Axis [kg-m2]\": 0.5 },\n"
            "  \"Enable Product of Inertia File\": true,\n"
            "  \"Product of Inertia File\": { \"Ixy File Path\": \"" + kCsvIxy + "\","
            " \"Ixz File Path\": \"" + kCsvIxz + "\", \"Iyz File Path\": \"" + kCsvIyz + "\" },\n"
            "  \"Enable X-C.P. File\": true,\n"
            "  \"X-C.P. File\": { \"X-C.P. File Path\": \"" + kCsvXcp + "\" },\n"
            "  \"Constant X-C.P.\": { \"Constant X-C.P. from BodyTail [mm]\": 835.0 },\n"
            "  \"X-ThrustLoadingPoint from BodyTail [mm]\": 250.0,\n"
            "  \"Enable CA File\": true,\n"
            "  \"CA File\": { \"CA File Path\": \"" + kCsvCA + "\", \"BurnOut CA File Path\": \"" + kCsvCAburn + "\" },\n"
            "  \"Constant CA\": { \"Constant CA [-]\": 0.40, \"Constant BurnOut CA [-]\": 0.50 },\n"
            "  \"Enable CNa File\": true,\n"
            "  \"CNa File\": { \"CNa File Path\": \"" + kCsvCNa + "\" },\n"
            "  \"Constant CNa\": { \"Constant CNa [1/rad]\": 10.0 },\n"
            "  \"Fin Cant Angle [deg]\": 1.0,\n"
            "  \"Enable Cld File\": true,\n"
            "  \"Cld File\": { \"Cld File Path\": \"" + kCsvCld + "\" },\n"
            "  \"Constant Cld\": { \"Constant Cld [1/rad]\": 0.0 },\n"
            "  \"Enable Clp File\": true,\n"
            "  \"Clp File\": { \"Clp File Path\": \"" + kCsvClp + "\" },\n"
            "  \"Constant Clp\": { \"Constant Clp [-]\": 0.03 },\n"
            "  \"Enable Cmq File\": true,\n"
            "  \"Cmq File\": { \"Cmq File Path\": \"" + kCsvCmq + "\" },\n"
            "  \"Constant Cmq\": { \"Constant Cmq [-]\": 7.0 },\n"
            "  \"Enable Cnr File\": true,\n"
            "  \"Cnr File\": { \"Cnr File Path\": \"" + kCsvCnr + "\" },\n"
            "  \"Constant Cnr\": { \"Constant Cnr [-]\": 7.0 }\n"
            "}\n");

        // --- Variant (3): Program Attitude Mode "Rate" + CONSTANT Product of
        //     Inertia (Enable Product of Inertia=true, File=false). Everything
        //     else mostly constant; no y/z thrust offset keys and no C.G.Offset
        //     block, and "Constant X-C.G." has no lateral keys -> exercises the
        //     legacy-else branch where neither contains() fires.
        WriteFile(kRocketRate,
            std::string("{\n"
            "  \"Diameter [mm]\": 150.0,\n"
            "  \"Length [mm]\": 2000.0,\n"
            "  \"Mass\": { \"Inert [kg]\": 10.0, \"Propellant [kg]\": 5.0 },\n"
            "  \"Enable Gas Jet\": false,\n"
            "  \"Gas Jet\": { \"Rolling Moment [N.m]\": 5.0, \"Duration [s]\": 2.5 },\n"
            "  \"Enable Program Attitude\": true,\n"
            "  \"Program Attitude\": { \"Mode\": \"Rate\", \"Enable Yaw\": true,"
            " \"Enable Pitch\": false, \"Enable Roll\": true, \"File Path\": \"") + kCsvAtt + "\" },\n"
            "  \"Enable X-C.G. File\": false,\n"
            "  \"X-C.G. File\": { \"X-C.G. File Path\": \"x.csv\" },\n"
            "  \"Constant X-C.G.\": { \"Constant X-C.G. from BodyTail [mm]\": 900.0 },\n"
            "  \"Enable M.I. File\": false,\n"
            "  \"M.I. File\": { \"M.I. File Path\": \"x.csv\" },\n"
            "  \"Constant M.I.\": { \"Yaw Axis [kg-m2]\": 5.0, \"Pitch Axis [kg-m2]\": 5.0, \"Roll Axis [kg-m2]\": 0.1 },\n"
            "  \"Enable Product of Inertia\": true,\n"
            "  \"Constant Product of Inertia\": { \"Ixy [kg-m2]\": 0.01, \"Ixz [kg-m2]\": 0.02, \"Iyz [kg-m2]\": 0.03 },\n"
            "  \"Enable X-C.P. File\": false,\n"
            "  \"X-C.P. File\": { \"X-C.P. File Path\": \"x.csv\" },\n"
            "  \"Constant X-C.P.\": { \"Constant X-C.P. from BodyTail [mm]\": 700.0 },\n"
            "  \"X-ThrustLoadingPoint from BodyTail [mm]\": 100.0,\n"
            "  \"Enable CA File\": false,\n"
            "  \"CA File\": { \"CA File Path\": \"x.csv\", \"BurnOut CA File Path\": \"x.csv\" },\n"
            "  \"Constant CA\": { \"Constant CA [-]\": 0.42, \"Constant BurnOut CA [-]\": 0.52 },\n"
            "  \"Enable CNa File\": false,\n"
            "  \"CNa File\": { \"CNa File Path\": \"x.csv\" },\n"
            "  \"Constant CNa\": { \"Constant CNa [1/rad]\": 9.0 },\n"
            "  \"Fin Cant Angle [deg]\": 0.0,\n"
            "  \"Enable Cld File\": false,\n"
            "  \"Cld File\": { \"Cld File Path\": \"x.csv\" },\n"
            "  \"Constant Cld\": { \"Constant Cld [1/rad]\": 0.0 },\n"
            "  \"Enable Clp File\": false,\n"
            "  \"Clp File\": { \"Clp File Path\": \"x.csv\" },\n"
            "  \"Constant Clp\": { \"Constant Clp [-]\": 0.03 },\n"
            "  \"Enable Cmq File\": false,\n"
            "  \"Cmq File\": { \"Cmq File Path\": \"x.csv\" },\n"
            "  \"Constant Cmq\": { \"Constant Cmq [-]\": 7.0 },\n"
            "  \"Enable Cnr File\": false,\n"
            "  \"Cnr File\": { \"Cnr File Path\": \"x.csv\" },\n"
            "  \"Constant Cnr\": { \"Constant Cnr [-]\": 7.0 }\n"
            "}\n");
    }

    void TearDown() override {
        const char* files[] = {
            kEngineJson, kRocketConst, kRocketFile, kRocketRate,
            kCsvXcg, kCsvXcp, kCsvCA, kCsvCAburn, kCsvCNa, kCsvCld,
            kCsvClp, kCsvCmq, kCsvCnr, kCsvIxy, kCsvIxz, kCsvIyz,
            kCsvMOI, kCsvAtt,
        };
        for (const char* f : files) std::remove(f);
    }

    RocketFactory factory;
};

// =====================================================================
// Variant 1: ALL CONSTANT branches + C.G. Offset block + thrust offsets.
// =====================================================================
TEST_F(RocketFactoryTest, AllConstantBranches) {
    Rocket r = factory.Create(kRocketConst, kEngineJson);

    // Scalars read straight off the JSON.
    EXPECT_NEAR(r.diameter, 0.180, 1e-12);             // 180 mm / 1e3
    EXPECT_NEAR(r.area, 0.25 * 0.180 * 0.180 * kPi, 1e-12);  // 0.25*d^2*pi
    EXPECT_NEAR(r.length, 3.900, 1e-12);               // 3900 mm / 1e3
    EXPECT_NEAR(r.mass.inert, 52.5, 1e-12);
    EXPECT_NEAR(r.mass.propellant, 41.0, 1e-12);

    // Gas jet disabled -> config not populated; enable flag false.
    EXPECT_FALSE(r.gas_jet_config.enable);

    // Program attitude disabled.
    EXPECT_FALSE(r.enable_program_attitude);

    // C.G. Offset block present -> y/z_CG set from it (mm/1e3).
    EXPECT_NEAR(r.y_CG, 0.012, 1e-12);                 //  12 mm / 1e3
    EXPECT_NEAR(r.z_CG, -0.008, 1e-12);                //  -8 mm / 1e3

    // Thrust loading point + lateral offsets present.
    EXPECT_NEAR(r.length_thrust, 0.300, 1e-12);        // 300 mm / 1e3
    EXPECT_NEAR(r.y_thrust_offset, 0.007, 1e-12);      //   7 mm / 1e3
    EXPECT_NEAR(r.z_thrust_offset, -0.003, 1e-12);     //  -3 mm / 1e3

    // Fin cant angle: 0 deg -> 0 rad.
    EXPECT_NEAR(r.cant_angle_fin, 0.0, 1e-12);

    // X-C.G. constant: getLengthCG() reads stored length_CG when NOT burning.
    // length_CG defaults to 0 after Create (set only when burning), so force
    // burning to read the constant src (1100 mm / 1e3 = 1.1 m at countup=0).
    r.engine.Ignittion();
    EXPECT_NEAR(r.getLengthCG(), 1.1, 1e-12);

    // X-C.P. constant: getLengthCP(mach) always reads src. 835 mm / 1e3.
    EXPECT_NEAR(r.getLengthCP(0.5), 0.835, 1e-12);

    // M.I. constant -> diagonal tensor (burning reads src; off-diag = -POI = 0).
    Eigen::Matrix3d It = r.getInertiaTensor();   // engine burning from Ignittion above
    EXPECT_NEAR(It(0, 0), 0.5, 1e-12);           // Roll Axis -> xx
    EXPECT_NEAR(It(1, 1), 46.0, 1e-12);          // Pitch Axis -> yy
    EXPECT_NEAR(It(2, 2), 45.0, 1e-12);          // Yaw Axis -> zz
    EXPECT_NEAR(It(0, 1), 0.0, 1e-12);           // -Ixy (POI off)
    EXPECT_NEAR(It(0, 2), 0.0, 1e-12);
    EXPECT_NEAR(It(1, 2), 0.0, 1e-12);

    // CA constant: burning -> CA_src (0.40); burnout -> CA_burnout_src (0.50).
    EXPECT_NEAR(r.getCA(0.5), 0.40, 1e-12);      // engine still burning
    r.engine.Cutoff();
    EXPECT_NEAR(r.getCA(0.5), 0.50, 1e-12);

    // CNa / Cld constants.
    EXPECT_NEAR(r.getCNa(0.5), 10.0, 1e-12);
    EXPECT_NEAR(r.getCld(0.5), 0.0, 1e-12);

    // Damping constants are forced <= 0 by the getters. Constant Clp/Cmq/Cnr
    // are positive in JSON (0.03 / 7 / 7) -> negated.
    EXPECT_NEAR(r.getClp(0.5), -0.03, 1e-12);
    EXPECT_NEAR(r.getCmq(0.5), -7.0, 1e-12);
    EXPECT_NEAR(r.getCnr(0.5), -7.0, 1e-12);
}

// =====================================================================
// Variant 2: ALL FILE branches + Gas Jet + Program Attitude(Angle) +
// Product-of-Inertia *File* + legacy lateral C.G. (no "C.G. Offset" block).
// =====================================================================
TEST_F(RocketFactoryTest, AllFileBranches) {
    Rocket r = factory.Create(kRocketFile, kEngineJson);

    EXPECT_NEAR(r.diameter, 0.200, 1e-12);             // 200 mm / 1e3
    EXPECT_NEAR(r.area, 0.25 * 0.200 * 0.200 * kPi, 1e-12);
    EXPECT_NEAR(r.length, 4.000, 1e-12);
    EXPECT_NEAR(r.mass.inert, 60.0, 1e-12);
    EXPECT_NEAR(r.mass.propellant, 30.0, 1e-12);

    // Gas jet enabled -> values read from "Gas Jet".
    EXPECT_TRUE(r.gas_jet_config.enable);
    EXPECT_NEAR(r.gas_jet_config.rolling_moment, 4.5, 1e-12);
    EXPECT_NEAR(r.gas_jet_config.duration, 3.0, 1e-12);

    // Program attitude enabled, Mode "Angle" -> mode_rate false.
    EXPECT_TRUE(r.enable_program_attitude);
    EXPECT_FALSE(r.attitude_program_config.mode_rate);
    EXPECT_FALSE(r.attitude_program_config.enable_yaw);   // JSON: false
    EXPECT_TRUE(r.attitude_program_config.enable_pitch);  // JSON: true
    EXPECT_FALSE(r.attitude_program_config.enable_roll);  // JSON: false
    // Attitude CSV times 0..10 -> control window.
    EXPECT_NEAR(r.time_start_attitude_control, 0.0, 1e-12);
    EXPECT_NEAR(r.time_end_attitude_control, 10.0, 1e-12);
    // Angle mode populated azimuth/elevation/roll program; CSV 90 deg -> pi/2 rad.
    Eigen::Vector3d att = r.getAttitude();
    EXPECT_NEAR(att(0), kPi / 2.0, 1e-9);
    EXPECT_NEAR(att(1), kPi / 2.0, 1e-9);
    EXPECT_NEAR(att(2), kPi / 2.0, 1e-9);

    // No "C.G. Offset" block -> legacy keys under "Constant X-C.G." used.
    EXPECT_NEAR(r.y_CG, 0.021, 1e-12);                 //  21 mm / 1e3
    EXPECT_NEAR(r.z_CG, -0.005, 1e-12);                //  -5 mm / 1e3

    // No y/z thrust offset keys present -> defaults (0.0) untouched.
    EXPECT_NEAR(r.length_thrust, 0.250, 1e-12);        // 250 mm / 1e3
    EXPECT_NEAR(r.y_thrust_offset, 0.0, 1e-12);
    EXPECT_NEAR(r.z_thrust_offset, 0.0, 1e-12);

    // Fin cant angle: 1 deg -> pi/180 rad.
    EXPECT_NEAR(r.cant_angle_fin, kPi / 180.0, 1e-12);

    // X-C.G. file at t=0 (countup) -> first row value 1.234 m (burning branch).
    r.engine.Ignittion();
    EXPECT_NEAR(r.getLengthCG(), 1.234, 1e-9);

    // X-C.P. file at mach=0 -> first row 0.876 m.
    EXPECT_NEAR(r.getLengthCP(0.0), 0.876, 1e-9);

    // M.I. file at t=0: yaw=40 -> zz, pitch=41 -> yy, roll=2 -> xx.
    // POI file at t=0: Ixy=0.07, Ixz=0.09, Iyz=0.11 -> off-diagonals are negated.
    Eigen::Matrix3d It = r.getInertiaTensor();   // burning
    EXPECT_NEAR(It(0, 0), 2.0, 1e-9);            // roll -> xx
    EXPECT_NEAR(It(1, 1), 41.0, 1e-9);           // pitch -> yy
    EXPECT_NEAR(It(2, 2), 40.0, 1e-9);           // yaw -> zz
    EXPECT_NEAR(It(0, 1), -0.07, 1e-9);          // -Ixy
    EXPECT_NEAR(It(1, 0), -0.07, 1e-9);
    EXPECT_NEAR(It(0, 2), -0.09, 1e-9);          // -Ixz
    EXPECT_NEAR(It(2, 0), -0.09, 1e-9);
    EXPECT_NEAR(It(1, 2), -0.11, 1e-9);          // -Iyz
    EXPECT_NEAR(It(2, 1), -0.11, 1e-9);

    // CA file: burning -> CA_src@mach0=0.41; burnout -> CA_burnout_src@mach0=0.55.
    EXPECT_NEAR(r.getCA(0.0), 0.41, 1e-9);       // burning
    r.engine.Cutoff();
    EXPECT_NEAR(r.getCA(0.0), 0.55, 1e-9);

    // CNa / Cld files at mach=0.
    EXPECT_NEAR(r.getCNa(0.0), 11.0, 1e-9);
    EXPECT_NEAR(r.getCld(0.0), 0.02, 1e-9);

    // Damping files already <=0 at mach0 -> returned unchanged (else branch).
    EXPECT_NEAR(r.getClp(0.0), -0.04, 1e-9);
    EXPECT_NEAR(r.getCmq(0.0), -6.0, 1e-9);
    EXPECT_NEAR(r.getCnr(0.0), -6.5, 1e-9);
}

// =====================================================================
// Variant 3: Program Attitude Mode "Rate" + CONSTANT Product of Inertia,
// no lateral C.G. keys at all (neither contains() block fires).
// =====================================================================
TEST_F(RocketFactoryTest, RateModeAndConstantProductOfInertia) {
    Rocket r = factory.Create(kRocketRate, kEngineJson);

    EXPECT_NEAR(r.diameter, 0.150, 1e-12);             // 150 mm / 1e3
    EXPECT_NEAR(r.mass.inert, 10.0, 1e-12);

    // Program attitude enabled, Mode "Rate" -> mode_rate true.
    EXPECT_TRUE(r.enable_program_attitude);
    EXPECT_TRUE(r.attitude_program_config.mode_rate);
    EXPECT_TRUE(r.attitude_program_config.enable_yaw);
    EXPECT_FALSE(r.attitude_program_config.enable_pitch);
    EXPECT_TRUE(r.attitude_program_config.enable_roll);
    // Rate mode populated the *rate* program (azimuth/elev/roll rate). CSV 90
    // deg/s -> pi/2 rad/s. getAttitude() (the angle program) stays zero.
    Eigen::Vector3d rate = r.getAttitudeRate();
    EXPECT_NEAR(rate(0), kPi / 2.0, 1e-9);
    EXPECT_NEAR(rate(1), kPi / 2.0, 1e-9);
    EXPECT_NEAR(rate(2), kPi / 2.0, 1e-9);
    Eigen::Vector3d ang = r.getAttitude();             // angle program never set -> 0
    EXPECT_NEAR(ang(0), 0.0, 1e-12);

    // No "C.G. Offset" block AND no lateral keys under "Constant X-C.G."
    // -> both contains() checks false -> y_CG/z_CG keep their defaults (0).
    EXPECT_NEAR(r.y_CG, 0.0, 1e-12);
    EXPECT_NEAR(r.z_CG, 0.0, 1e-12);

    // No y/z thrust offset keys -> default 0.
    EXPECT_NEAR(r.length_thrust, 0.100, 1e-12);        // 100 mm / 1e3
    EXPECT_NEAR(r.y_thrust_offset, 0.0, 1e-12);
    EXPECT_NEAR(r.z_thrust_offset, 0.0, 1e-12);

    // Constant Product of Inertia -> off-diagonals = -POI from JSON.
    r.engine.Ignittion();
    Eigen::Matrix3d It = r.getInertiaTensor();
    EXPECT_NEAR(It(0, 0), 0.1, 1e-12);                 // roll -> xx
    EXPECT_NEAR(It(1, 1), 5.0, 1e-12);                 // pitch -> yy
    EXPECT_NEAR(It(2, 2), 5.0, 1e-12);                 // yaw -> zz
    EXPECT_NEAR(It(0, 1), -0.01, 1e-12);               // -Ixy
    EXPECT_NEAR(It(0, 2), -0.02, 1e-12);               // -Ixz
    EXPECT_NEAR(It(1, 2), -0.03, 1e-12);               // -Iyz

    // Constant X-C.G. (no lateral keys) still resolves to 900 mm / 1e3 = 0.9 m.
    EXPECT_NEAR(r.getLengthCG(), 0.9, 1e-12);          // burning
}

}  // namespace
}  // namespace forrocket
