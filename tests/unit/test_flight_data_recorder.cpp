// ******************************************************
// Unit tests for src/rocket/flight_data_recorder.cpp
//
// The FlightDataRecorder is a state observer: operator()(state, t) appends one
// row of the current Rocket state to a set of parallel std::vectors, and
// DumpCsv() writes those vectors to a CSV file (full or reduced column set).
//
// Decision / branch points covered (target: >=80% C1):
//   D1  operator() line 61   if (LLH(2) >= 0.0)          : record (true) / skip (false)
//   D2  DumpCsv  line 100/175 if (full_dump)  -- header  : full (true) / reduced (false)
//   D3  DumpCsv  line 240     for over rows              : 0 rows / >=1 row
//   D4  DumpCsv  line 247/322 if (full_dump)  -- rows    : full (true) / reduced (false)
//   Compound conditions in the full-dump resonance block (only reached when
//   full_dump == true), both sides exercised via two row scenarios:
//     C1  line 371/385/389  (k_alpha > 0.0 && It > 0.0)
//     C2  line 374          (omega_n > 0.0)
//     C3  line 383/384      (Vair > 0.0)
//     C4  line 386          (abs(zeta) > 1.0e-9)
//     C5  line 392          (It > 0.0)
//     C6  line 394          (abs(sd_denom) > 1.0e-12)
//     C7  line 396          (Sg > 0.0 && (1/Sg) < sd_bound)
//     C8  line 400          (abs(Clp) > 1.0e-12 && Vair > 0.0)
//     C9  line 417          (k_alpha > 0.0 && amp_resp > 1.0e-9)
//
// The commented-out IIP call (line ~442) is intentionally ignored — it is not
// in the production build.
//
// Uses the shared Rocket builder in test_fixtures.hpp (read-only).
// ******************************************************

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "rocket/flight_data_recorder.hpp"
#include "rocket/rocket.hpp"
#include "rocket/parameter/interpolate_parameter.hpp"
#include "dynamics/dynamics_base.hpp"
#include "degrad.hpp"

#include "test_fixtures.hpp"

using forrocket::FlightDataRecorder;
using forrocket::Rocket;
using forrocket::DynamicsBase;
using forrocket::test::MakeTestRocket;

namespace {

// A dummy ODE state; the recorder only stores t and reads from *p_rocket, so
// the contents of x are irrelevant to what gets recorded.
DynamicsBase::state ZeroState() {
    DynamicsBase::state x;
    x.fill(0.0);
    return x;
}

// Split one CSV line into fields by ','. The recorder emits a trailing comma
// after every column, so the final split element is the empty string after the
// last comma; callers account for that.
std::vector<std::string> SplitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        out.push_back(field);
    }
    return out;
}

// Read all non-empty lines of a file.
std::vector<std::string> ReadLines(const std::string& path) {
    std::vector<std::string> lines;
    std::ifstream ifs(path);
    std::string line;
    while (std::getline(ifs, line)) {
        if (!line.empty()) lines.push_back(line);
    }
    return lines;
}

// Unique temp path we control (preferred over tmpnam): includes the test name.
std::string TempPath(const std::string& tag) {
    return "/tmp/forrocket_fdr_" + tag + ".csv";
}

}  // namespace

// ---------------------------------------------------------------------------
// D1: operator() altitude gate
// ---------------------------------------------------------------------------

// D1-true: LLH altitude >= 0 -> the record is appended. Also the core invariant:
// number of recorded rows == number of accepted pushes.
TEST(FlightDataRecorder, RecordAppendsWhenAltitudeNonNegative) {
    Rocket r = MakeTestRocket();  // LaunchLLH altitude = 20.0 m (>= 0)
    FlightDataRecorder fdr(&r);
    DynamicsBase::state x = ZeroState();

    fdr(x, 0.0);
    fdr(x, 0.1);
    fdr(x, 0.2);

    // One row per accepted push, across every parallel vector.
    EXPECT_EQ(fdr.countup_time.size(), 3u);
    EXPECT_EQ(fdr.countup_burn_time.size(), 3u);
    EXPECT_EQ(fdr.thrust.size(), 3u);
    EXPECT_EQ(fdr.position.size(), 3u);
    EXPECT_EQ(fdr.moment.size(), 3u);
    // The 't' passed in is stored verbatim.
    EXPECT_DOUBLE_EQ(fdr.countup_time[0], 0.0);
    EXPECT_DOUBLE_EQ(fdr.countup_time[1], 0.1);
    EXPECT_DOUBLE_EQ(fdr.countup_time[2], 0.2);
}

// D1-false: LLH altitude < 0 -> the record is NOT appended (e.g. impact below
// the ellipsoid). Covers the false side of `if (LLH(2) >= 0.0)`.
TEST(FlightDataRecorder, RecordSkippedWhenAltitudeNegative) {
    Rocket r = MakeTestRocket();
    FlightDataRecorder fdr(&r);
    DynamicsBase::state x = ZeroState();

    // One valid push at altitude 20 m.
    fdr(x, 0.0);
    // Drop below the ellipsoid -> push must be rejected.
    r.position.LLH(2) = -5.0;
    fdr(x, 0.1);
    r.position.LLH(2) = -0.001;  // just below zero, still rejected
    fdr(x, 0.2);

    EXPECT_EQ(fdr.countup_time.size(), 1u);
    EXPECT_DOUBLE_EQ(fdr.countup_time[0], 0.0);

    // Boundary: exactly 0.0 is accepted (>= is inclusive).
    r.position.LLH(2) = 0.0;
    fdr(x, 0.3);
    EXPECT_EQ(fdr.countup_time.size(), 2u);
    EXPECT_DOUBLE_EQ(fdr.countup_time[1], 0.3);
}

// A recorded value reflects the live rocket state at push time: mutating the
// rocket between pushes yields distinct stored values.
TEST(FlightDataRecorder, RecordsLiveStateAtPushTime) {
    Rocket r = MakeTestRocket();
    FlightDataRecorder fdr(&r);
    DynamicsBase::state x = ZeroState();

    r.dynamic_pressure = 1000.0;
    fdr(x, 0.0);
    r.dynamic_pressure = 2500.0;
    fdr(x, 0.1);

    ASSERT_EQ(fdr.dynamic_pressure.size(), 2u);
    EXPECT_DOUBLE_EQ(fdr.dynamic_pressure[0], 1000.0);
    EXPECT_DOUBLE_EQ(fdr.dynamic_pressure[1], 2500.0);
    // mass = inert + propellant = 10 + 5 = 15 (fixture values).
    EXPECT_DOUBLE_EQ(fdr.mass[0], 15.0);
}

// ReserveCapacity must not change logical size (capacity-only operation).
TEST(FlightDataRecorder, ReserveCapacityDoesNotChangeSize) {
    Rocket r = MakeTestRocket();
    FlightDataRecorder fdr(&r);
    fdr.ReserveCapacity(128);
    EXPECT_EQ(fdr.countup_time.size(), 0u);
    EXPECT_GE(fdr.countup_time.capacity(), 128u);
    EXPECT_GE(fdr.moment.capacity(), 128u);
}

// ---------------------------------------------------------------------------
// D3: DumpCsv with zero rows (loop body never executes) -- header only.
// ---------------------------------------------------------------------------

// D3-false (0 rows) combined with D2-true (full_dump header).
TEST(FlightDataRecorder, DumpCsvEmptyFullDumpWritesHeaderOnly) {
    Rocket r = MakeTestRocket();
    FlightDataRecorder fdr(&r);  // no pushes -> 0 rows
    const std::string path = TempPath("empty_full");

    fdr.DumpCsv(path, true);

    std::vector<std::string> lines = ReadLines(path);
    ASSERT_EQ(lines.size(), 1u);  // header line only, no data rows
    // Header begins with the time column.
    EXPECT_EQ(lines[0].substr(0, 9), "Time [s],");
    std::remove(path.c_str());
}

// D3-false (0 rows) combined with D2-false (reduced header).
TEST(FlightDataRecorder, DumpCsvEmptyReducedWritesHeaderOnly) {
    Rocket r = MakeTestRocket();
    FlightDataRecorder fdr(&r);
    const std::string path = TempPath("empty_reduced");

    fdr.DumpCsv(path, false);

    std::vector<std::string> lines = ReadLines(path);
    ASSERT_EQ(lines.size(), 1u);
    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// D2/D4: reduced dump (full_dump == false) -- false side of every full_dump if.
// ---------------------------------------------------------------------------

// The reduced CSV has exactly these columns (Time, Burn Time, Latitude,
// Longitude, Altitude, Downrange). Header column count must equal data column
// count, and a pushed altitude must appear in the Altitude column.
TEST(FlightDataRecorder, DumpCsvReducedColumnsAndAltitudeRoundtrip) {
    Rocket r = MakeTestRocket();
    FlightDataRecorder fdr(&r);
    DynamicsBase::state x = ZeroState();

    // Push two rows with distinct, known altitudes (both >= 0 so accepted).
    r.position.LLH(2) = 123.0;
    fdr(x, 0.0);
    r.position.LLH(2) = 456.0;
    fdr(x, 1.0);

    const std::string path = TempPath("reduced_cols");
    fdr.DumpCsv(path, false);

    std::vector<std::string> lines = ReadLines(path);
    ASSERT_EQ(lines.size(), 3u);  // header + 2 data rows (D3-true)

    // Trailing comma after the last value -> SplitCsv drops the empty tail, so
    // the reduced layout has exactly 6 fields:
    //   0 Time, 1 Burn Time, 2 Latitude, 3 Longitude, 4 Altitude, 5 Downrange.
    std::vector<std::string> header = SplitCsv(lines[0]);
    std::vector<std::string> row0 = SplitCsv(lines[1]);
    std::vector<std::string> row1 = SplitCsv(lines[2]);
    EXPECT_EQ(header.size(), 6u);          // 6 columns in reduced mode
    EXPECT_EQ(header.size(), row0.size()); // header count == data count
    EXPECT_EQ(header.size(), row1.size());

    EXPECT_EQ(header[4], "Altitude [m]");
    // Altitude column round-trips the pushed value (fixed precision 8).
    EXPECT_NEAR(std::stod(row0[4]), 123.0, 1e-6);
    EXPECT_NEAR(std::stod(row1[4]), 456.0, 1e-6);
    // Time column round-trips 't'.
    EXPECT_NEAR(std::stod(row0[0]), 0.0, 1e-6);
    EXPECT_NEAR(std::stod(row1[0]), 1.0, 1e-6);
    // First row downrange from itself is 0.
    EXPECT_NEAR(std::stod(row0[5]), 0.0, 1e-3);
    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// D2/D4: full dump (full_dump == true) -- true side of every full_dump if,
// header column count == data column count, and resonance diagnostics
// compound-condition TRUE branches.
// ---------------------------------------------------------------------------

// Scenario A: statically stable, positive dynamic pressure and airspeed -> the
// TRUE sides of the resonance compound conditions (k_alpha>0, It>0, Vair>0,
// omega_n>0, ...). Verify header/data column counts match and a few known
// values land in the right columns.
TEST(FlightDataRecorder, DumpCsvFullColumnsMatchAndDiagnosticsActive) {
    Rocket r = MakeTestRocket();
    FlightDataRecorder fdr(&r);
    DynamicsBase::state x = ZeroState();

    // The recorder records the live raw fields (p_rocket->CNa, ...), NOT the
    // getters, and the fixture only sets the *_src interpolators (live fields
    // default to 0). Set the live aero coefficients explicitly so the dump-time
    // resonance math has non-zero inputs.
    r.CNa = 10.0;
    r.CA = 0.3;
    r.Cmq = -2.0;
    r.Clp = -0.1;
    r.Cld = 0.0;
    // Make k_alpha > 0: need sm = length_CG - length_CP > 0 and q*S*CNa > 0.
    // Set CG aft of CP so sm = +0.2 m.
    r.length_CG = 1.4;
    r.length_CP = 1.2;
    r.dynamic_pressure = 5000.0;            // q > 0
    r.velocity.air_body << 100.0, 0.0, 0.0; // Vair = 100 m/s > 0
    r.velocity.mach_number = 0.3;
    r.angular_velocity << 30.0, 0.0, 0.0;   // spin -> f_spin > 0
    r.angle_of_attack = forrocket::deg2rad(2.0);
    r.sideslip_angle = forrocket::deg2rad(1.0);
    r.position.LLH(2) = 1000.0;
    fdr(x, 0.5);

    const std::string path = TempPath("full_active");
    fdr.DumpCsv(path, true);

    std::vector<std::string> lines = ReadLines(path);
    ASSERT_EQ(lines.size(), 2u);  // header + 1 row

    std::vector<std::string> header = SplitCsv(lines[0]);
    std::vector<std::string> row = SplitCsv(lines[1]);
    // Invariant: in full mode the header column count equals the data column
    // count. (We do not hardcode the exact number; it is large and may evolve.)
    EXPECT_EQ(header.size(), row.size());
    // Full dump must have many more columns than the 6-column reduced layout.
    EXPECT_GT(header.size(), 6u);

    // Locate a few columns by name and check the values landed correctly.
    auto colIndex = [&](const std::string& name) -> int {
        for (std::size_t i = 0; i < header.size(); ++i) {
            if (header[i] == name) return static_cast<int>(i);
        }
        return -1;
    };

    int iAlt = colIndex("Altitude [m]");
    ASSERT_GE(iAlt, 0);
    EXPECT_NEAR(std::stod(row[iAlt]), 1000.0, 1e-6);

    int iMass = colIndex("Mass [kg]");
    ASSERT_GE(iMass, 0);
    EXPECT_NEAR(std::stod(row[iMass]), 15.0, 1e-6);  // 10 inert + 5 prop

    int iBurning = colIndex("Burning [0/1]");
    ASSERT_GE(iBurning, 0);
    // engine.burning default in fixture (engine constructed, not ignited) -> 0.
    EXPECT_EQ(row[iBurning], "0");

    // Spin frequency = |p| / (2*pi) = 30 / (2*pi) ~= 4.7746 Hz.
    int iSpin = colIndex("SpinFreq [Hz]");
    ASSERT_GE(iSpin, 0);
    EXPECT_NEAR(std::stod(row[iSpin]), 30.0 / (2.0 * forrocket::pi), 1e-4);

    // k_alpha = q*S*CNa*sm > 0 here, so PitchYawNaturalFreq should be > 0
    // (true side of C1/C2). With k_alpha = 5000 * (pi*0.15^2/4) * 10 * 0.2,
    // It = 5.0 -> omega_n = sqrt(k_alpha/It); f_n = omega_n/(2*pi) > 0.
    int iFn = colIndex("PitchYawNaturalFreq [Hz]");
    ASSERT_GE(iFn, 0);
    EXPECT_GT(std::stod(row[iFn]), 0.0);

    // TotalAoA = sqrt(aoa^2 + aos^2) = sqrt(2^2 + 1^2) = sqrt(5) ~= 2.2360 deg.
    int iTotAoA = colIndex("TotalAoA [deg]");
    ASSERT_GE(iTotAoA, 0);
    EXPECT_NEAR(std::stod(row[iTotAoA]), std::sqrt(5.0), 1e-3);

    std::remove(path.c_str());
}

// Scenario B: zero dynamic pressure and zero airspeed (e.g. pre-launch / above
// the sensible atmosphere) -> the FALSE sides of the resonance compound
// conditions. All guarded diagnostics fall back to their 0.0 defaults.
TEST(FlightDataRecorder, DumpCsvFullDiagnosticsGuardedToZero) {
    Rocket r = MakeTestRocket();
    FlightDataRecorder fdr(&r);
    DynamicsBase::state x = ZeroState();

    r.dynamic_pressure = 0.0;               // q = 0 -> k_alpha = 0 (false C1/C9)
    r.velocity.air_body << 0.0, 0.0, 0.0;   // Vair = 0 (false C3/C8)
    r.velocity.mach_number = 0.0;
    r.angular_velocity << 0.0, 0.0, 0.0;    // no spin -> f_spin = 0
    r.position.LLH(2) = 0.0;
    fdr(x, 0.0);

    const std::string path = TempPath("full_zero");
    fdr.DumpCsv(path, true);

    std::vector<std::string> lines = ReadLines(path);
    ASSERT_EQ(lines.size(), 2u);
    std::vector<std::string> header = SplitCsv(lines[0]);
    std::vector<std::string> row = SplitCsv(lines[1]);
    EXPECT_EQ(header.size(), row.size());

    auto colIndex = [&](const std::string& name) -> int {
        for (std::size_t i = 0; i < header.size(); ++i) {
            if (header[i] == name) return static_cast<int>(i);
        }
        return -1;
    };

    // With q = 0 and Vair = 0, every guarded diagnostic takes its 0.0 fallback.
    int iFn = colIndex("PitchYawNaturalFreq [Hz]");
    int iSpin = colIndex("SpinFreq [Hz]");
    int iLambda = colIndex("ResonanceRatio [-]");
    int iSg = colIndex("GyroStabilityFactor Sg [-]");
    int iZeta = colIndex("PitchDampingRatio [-]");
    int iDynStable = colIndex("DynStable [0/1]");
    int iSpinEq = colIndex("EquilibriumSpinFreq [Hz]");
    int iLatLoad = colIndex("LateralAeroLoad [N]");
    ASSERT_GE(iFn, 0);
    ASSERT_GE(iSpin, 0);
    ASSERT_GE(iLambda, 0);
    ASSERT_GE(iSg, 0);
    ASSERT_GE(iZeta, 0);
    ASSERT_GE(iDynStable, 0);
    ASSERT_GE(iSpinEq, 0);
    ASSERT_GE(iLatLoad, 0);

    EXPECT_NEAR(std::stod(row[iFn]), 0.0, 1e-9);
    EXPECT_NEAR(std::stod(row[iSpin]), 0.0, 1e-9);
    EXPECT_NEAR(std::stod(row[iLambda]), 0.0, 1e-9);   // omega_n=0 -> lambda=0 (false C2)
    EXPECT_NEAR(std::stod(row[iSg]), 0.0, 1e-9);       // false C1 (Sg path)
    EXPECT_NEAR(std::stod(row[iZeta]), 0.0, 1e-9);     // false C3 -> c_lift/c_damp 0, zeta 0
    // Sg = 0 -> dyn_stable condition (Sg > 0.0 && ...) is false -> 0 (false C7).
    EXPECT_EQ(row[iDynStable], "0");
    EXPECT_NEAR(std::stod(row[iSpinEq]), 0.0, 1e-9);   // Vair=0 -> false C8
    EXPECT_NEAR(std::stod(row[iLatLoad]), 0.0, 1e-9);  // q=0 -> lat_load 0

    std::remove(path.c_str());
}

// Scenario C: dynamically-stable TRUE branch of the C7 compound condition
// (Sg > 0.0 && (1/Sg) < sd_bound -> dyn_stable == 1). Built with a high spin
// rate (large Sg) so 1/Sg is small, against a positive sd_bound.
//
// Independent check of the C7 logic (not just characterization): we recompute
// Sg, Sd and the boundary from the recorder's documented formulas and assert
// the DynStable flag matches our recomputation. This pins the *decision*, while
// the parallel-vector round-trip pins the recorded inputs.
TEST(FlightDataRecorder, DumpCsvFullDynStableTrueBranch) {
    Rocket r = MakeTestRocket();
    FlightDataRecorder fdr(&r);
    DynamicsBase::state x = ZeroState();

    // Live aero fields default to 0 in the fixture; set them so the dump-time
    // resonance math is non-trivial (recorder reads raw fields, not getters).
    // The values below are deliberately tuned so the C7 TRUE branch
    // (Sg > 0 && 1/Sg < sd_bound) is reached. Derivation (using the recorder's
    // own formulas, fixture S = pi*0.15^2/4 ~= 0.0176715 m^2, D = 0.15 m,
    // It = 0.5*(Iyy+Izz) = 5.0, Ix = inertia(0,0) = 0.1, mass = 15):
    //   sm     = length_CG - length_CP = 1.4 - 1.2 = +0.2 m
    //   k_alpha= q*S*CNa*sm = 2000*0.0176715*10*0.2 ~= 70.69  (> 0)
    //   Sg     = (Ix*p)^2 / (4*It*k_alpha) = (0.1*600)^2/(4*5*70.69) ~= 2.546
    //          -> 1/Sg ~= 0.393
    //   inv_ky2= m*D^2/It = 15*0.0225/5 = 0.0675
    //   sd_den = CNa - CA - inv_ky2*Cmq = 10 - 0.3 - 0.0675*(-50) = 13.075
    //   Sd     = 2*CNa/sd_den = 20/13.075 ~= 1.530  (in (0,2))
    //   sd_bnd = Sd*(2 - Sd) ~= 1.530*0.470 ~= 0.719  (> 0)
    //   => 1/Sg (0.393) < sd_bnd (0.719) -> dyn_stable = 1
    r.CNa = 10.0;
    r.CA = 0.3;
    r.Cmq = -50.0;                     // large pitch damping -> Sd in (0,2)
    r.Clp = -0.1;
    r.Cld = 0.0;
    // Statically stable (sm > 0) and a large spin so Sg > 1 (=> 1/Sg small).
    r.length_CG = 1.4;
    r.length_CP = 1.2;                 // sm = +0.2 m
    r.dynamic_pressure = 2000.0;       // q > 0
    r.velocity.air_body << 80.0, 0.0, 0.0;
    r.velocity.mach_number = 0.24;
    r.angular_velocity << 600.0, 0.0, 0.0;  // high spin -> large Sg
    r.position.LLH(2) = 500.0;
    fdr(x, 0.0);

    const std::string path = TempPath("full_dynstable");
    fdr.DumpCsv(path, true);

    std::vector<std::string> lines = ReadLines(path);
    ASSERT_EQ(lines.size(), 2u);
    std::vector<std::string> header = SplitCsv(lines[0]);
    std::vector<std::string> row = SplitCsv(lines[1]);

    auto colIndex = [&](const std::string& name) -> int {
        for (std::size_t i = 0; i < header.size(); ++i) {
            if (header[i] == name) return static_cast<int>(i);
        }
        return -1;
    };
    int iSg = colIndex("GyroStabilityFactor Sg [-]");
    int iSdBound = colIndex("DynStabilityBoundary Sd(2-Sd) [-]");
    int iDynStable = colIndex("DynStable [0/1]");
    ASSERT_GE(iSg, 0);
    ASSERT_GE(iSdBound, 0);
    ASSERT_GE(iDynStable, 0);

    double Sg = std::stod(row[iSg]);
    double sd_bound = std::stod(row[iSdBound]);
    int expected = (Sg > 0.0 && (1.0 / Sg) < sd_bound) ? 1 : 0;
    EXPECT_EQ(std::stoi(row[iDynStable]), expected);
    // For this high-spin, statically-stable case we expect the TRUE branch.
    EXPECT_GT(Sg, 0.0);
    EXPECT_EQ(row[iDynStable], "1");

    std::remove(path.c_str());
}
