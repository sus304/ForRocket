// ******************************************************
// Supplementary coverage for FlightDynamics::Compute6dofProgramRate — drives
// both modes (rate / angle) with every attitude axis both enabled and
// disabled, so both sides of each per-axis enable conditional are exercised.
// ******************************************************

#include <gtest/gtest.h>
#include <array>
#include <cmath>

#include "Eigen/Dense"

#include "degrad.hpp"
#include "dynamics/flight_dynamics.hpp"
#include "environment/datetime.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"
#include "rocket/parameter/interpolate_parameter.hpp"

#include "test_fixtures.hpp"

using forrocket::FlightDynamics;
using forrocket::Rocket;
using forrocket::SequenceClock;
using forrocket::EnvironmentWind;
using forrocket::InterpolateParameter;
using forrocket::DateTime;
using forrocket::deg2rad;
using forrocket::test::MakeTestRocket;
using forrocket::test::MakeZeroWind;

namespace {

FlightDynamics::state StateFrom(const Rocket& r) {
    FlightDynamics::state x;
    for (int i = 0; i < 3; ++i) x[i]      = r.position.ECI[i];
    for (int i = 0; i < 3; ++i) x[3 + i]  = r.velocity.ECI[i];
    for (int i = 0; i < 4; ++i) x[6 + i]  = r.attitude.quaternion[i];
    for (int i = 0; i < 3; ++i) x[10 + i] = r.angular_velocity[i];
    x[13] = r.mass.propellant;
    return x;
}

// Run the program-attitude path once with the given mode and axis-enable flags.
void RunProgram(bool mode_rate, bool yaw, bool pitch, bool roll) {
    Rocket r = MakeTestRocket();
    r.enable_program_attitude = true;
    r.time_start_attitude_control = 0.0;
    r.time_end_attitude_control = 100.0;
    r.attitude_program_config.mode_rate = mode_rate;
    r.attitude_program_config.enable_yaw = yaw;
    r.attitude_program_config.enable_pitch = pitch;
    r.attitude_program_config.enable_roll = roll;
    r.setAttitudeProgram(InterpolateParameter(deg2rad(270.0)),
                         InterpolateParameter(deg2rad(80.0)),
                         InterpolateParameter(0.0));
    r.setAttitudeProgramRate(InterpolateParameter(0.01),
                             InterpolateParameter(0.02),
                             InterpolateParameter(0.0));

    SequenceClock clock(DateTime(), 1.0);
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kInAir);

    FlightDynamics::state x = StateFrom(r);
    FlightDynamics::state dx;
    dx.fill(0.0);
    fd(x, dx, 1.0);  // t=1.0 is inside [start,end) and program is enabled

    for (double v : dx) EXPECT_TRUE(std::isfinite(v));
}

}  // namespace

TEST(FlightDynamicsProgram, RateModeAllAxesEnabled)   { RunProgram(true,  true,  true,  true);  }
TEST(FlightDynamicsProgram, RateModeAllAxesDisabled)  { RunProgram(true,  false, false, false); }
TEST(FlightDynamicsProgram, AngleModeAllAxesEnabled)  { RunProgram(false, true,  true,  true);  }
TEST(FlightDynamicsProgram, AngleModeAllAxesDisabled) { RunProgram(false, false, false, false); }
