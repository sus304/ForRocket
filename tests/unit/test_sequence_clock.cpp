// ******************************************************
// Unit tests for src/environment/sequence_clock.cpp
//   - SequenceClock constructors (default / UTC / UTC+ref-time)
//   - SyncSolverTime(): advances countup_time and recomputes
//     Julian date, Modified Julian date, Greenwich sidereal time
//
// The conversion helpers (UTC2JulianDate, JulianDate2ModifiedJulianDate,
// JulianDate2GreenwichSiderealTime) are private; they are exercised
// indirectly through the public members julian_data, modified_julian_date,
// greenwich_sidereal_time.
//
// Reference strategy: independent astronomical constants.
//   * J2000 epoch (2000-01-01 12:00:00 UTC)  -> Julian Date 2451545.0
//   * 1970-01-01 00:00 UTC                    -> Julian Date 2440587.5
//   * Modified Julian Date = Julian Date - 2400000.5
//   * Mean sidereal time at J2000 epoch        ~ 280.4606 deg
//     (IAU GMST at J2000 = 280.46061838 deg)
//
// Decision points exercised:
//   D1  default ctor path (UpdateJulianDate from J2000)
//   D2  UTC-only ctor (countup_time_ref defaults to 0 via init)
//   D3  UTC + countup_time_init ctor (nonzero reference offset)
//   D4  SyncSolverTime subtracts countup_time_ref
//   D5  GST fmod wrap into [0, 2*pi)
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>

#include "environment/sequence_clock.hpp"
#include "environment/datetime.hpp"

using forrocket::SequenceClock;
using forrocket::DateTime;

namespace {
const double kPi = 3.141592653589793;
const double kJD_J2000 = 2451545.0;       // JD at 2000-01-01 12:00:00 UTC
const double kJD_UnixEpoch = 2440587.5;   // JD at 1970-01-01 00:00:00 UTC
const double kMJD_offset = 2400000.5;     // JD - MJD
}  // namespace

// D1: default ctor uses DateTime() == J2000 noon.
// JD must equal the canonical J2000 value 2451545.0.
TEST(SequenceClock, DefaultConstructorIsJ2000) {
    SequenceClock clk;
    EXPECT_NEAR(clk.julian_data, kJD_J2000, 1e-6);
    EXPECT_DOUBLE_EQ(clk.countup_time, 0.0);
    // MJD = JD - 2400000.5 = 51544.5
    EXPECT_NEAR(clk.modified_julian_date, kJD_J2000 - kMJD_offset, 1e-6);
}

// D1/D5: Greenwich sidereal time at J2000 epoch.
// IAU mean sidereal time at J2000 = 280.46061838 deg. The implementation
// returns radians wrapped into [0, 2*pi).
TEST(SequenceClock, GreenwichSiderealTimeAtJ2000) {
    SequenceClock clk;  // J2000 noon
    double gst_deg = clk.greenwich_sidereal_time * 180.0 / kPi;
    // Source: IAU GMST(J2000) = 280.46061838 deg. Tolerance loose because the
    // implementation uses the truncated polynomial form.
    EXPECT_NEAR(gst_deg, 280.4606, 1e-2);
    // D5: result must be wrapped into [0, 2*pi).
    EXPECT_GE(clk.greenwich_sidereal_time, 0.0);
    EXPECT_LT(clk.greenwich_sidereal_time, 2.0 * kPi);
}

// D2: UTC-only ctor with the Unix epoch -> JD 2440587.5.
TEST(SequenceClock, UnixEpochJulianDate) {
    DateTime epoch(1970, 1, 1, 0, 0, 0);
    SequenceClock clk(epoch);
    EXPECT_NEAR(clk.julian_data, kJD_UnixEpoch, 1e-6);
    EXPECT_NEAR(clk.modified_julian_date, kJD_UnixEpoch - kMJD_offset, 1e-6);
}

// D2: half a day past midnight (12:00) adds exactly 0.5 to the JD.
TEST(SequenceClock, NoonAddsHalfDay) {
    SequenceClock midnight(DateTime(2000, 1, 1, 0, 0, 0));
    SequenceClock noon(DateTime(2000, 1, 1, 12, 0, 0));
    EXPECT_NEAR(noon.julian_data - midnight.julian_data, 0.5, 1e-9);
    // 2000-01-01 00:00 UTC -> JD 2451544.5 (J2000 minus half a day).
    EXPECT_NEAR(midnight.julian_data, kJD_J2000 - 0.5, 1e-6);
}

// D4: SyncSolverTime with countup_time_ref == 0 sets countup_time = t and
// advances the clock. 12 h (43200 s) from J2000 noon -> next midnight,
// JD = 2451545.5.
TEST(SequenceClock, SyncSolverTimeAdvancesClock) {
    SequenceClock clk;  // J2000 noon, ref = 0
    clk.SyncSolverTime(43200.0);  // +12 h
    EXPECT_DOUBLE_EQ(clk.countup_time, 43200.0);
    EXPECT_NEAR(clk.julian_data, kJD_J2000 + 0.5, 1e-6);
}

// D3 + D4: ctor with a nonzero countup_time_ref. SyncSolverTime(t) stores
// countup_time = t - ref, so calling with t == ref leaves the clock at the
// initial epoch.
TEST(SequenceClock, CountupReferenceOffsetIsSubtracted) {
    DateTime init(2000, 1, 1, 12, 0, 0);
    double ref = 100.0;
    SequenceClock clk(init, ref);  // D3
    // Right after construction countup_time is 0 and JD == J2000.
    EXPECT_DOUBLE_EQ(clk.countup_time, 0.0);
    EXPECT_NEAR(clk.julian_data, kJD_J2000, 1e-6);

    // D4: Sync at t == ref keeps us at the init epoch (countup_time == 0).
    clk.SyncSolverTime(ref);
    EXPECT_DOUBLE_EQ(clk.countup_time, 0.0);
    EXPECT_NEAR(clk.julian_data, kJD_J2000, 1e-6);

    // Sync at t = ref + 43200 advances exactly 12 h.
    clk.SyncSolverTime(ref + 43200.0);
    EXPECT_DOUBLE_EQ(clk.countup_time, 43200.0);
    EXPECT_NEAR(clk.julian_data, kJD_J2000 + 0.5, 1e-6);
}

// Monotonicity: a later solver time yields a strictly larger Julian date.
TEST(SequenceClock, JulianDateIsMonotonicInTime) {
    SequenceClock clk;
    clk.SyncSolverTime(10.0);
    double jd1 = clk.julian_data;
    clk.SyncSolverTime(20.0);
    double jd2 = clk.julian_data;
    EXPECT_GT(jd2, jd1);
}

// GST stays wrapped in [0, 2*pi) after a large advance (D5 wrap branch).
TEST(SequenceClock, GstStaysWrappedAfterLongAdvance) {
    SequenceClock clk;
    clk.SyncSolverTime(5.0 * 86400.0);  // +5 days
    EXPECT_GE(clk.greenwich_sidereal_time, 0.0);
    EXPECT_LT(clk.greenwich_sidereal_time, 2.0 * kPi);
}
