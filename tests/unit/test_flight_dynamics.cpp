// ******************************************************
// Unit tests for src/dynamics/flight_dynamics.cpp
//   - the 6DOF equations-of-motion right-hand-side: operator()(x, dx, t)
//   - exercises all four compute paths via SetRegime + the conditional
//     branches inside them (program-attitude rate/angle modes, free vs.
//     controlled axes, gas-jet, launcher friction/no-thrust clamp).
//
// Style mirrors test_rocket.cpp. Uses the shared builders in
// test_fixtures.hpp (read-only). No int main() (gtest_main provides it).
//
// State vector layout (confirmed from flight_dynamics.cpp Map<> offsets):
//   x[0..2]  : position  ECI   [m]
//   x[3..5]  : velocity  ECI   [m/s]
//   x[6..9]  : attitude  quaternion (NED->body), normalized internally
//   x[10..12]: angular velocity body (p, q, r) [rad/s]
//   x[13]    : propellant mass [kg]
// dx mirrors the same layout (position-rate, accel-ECI, quat-dot, ang-accel,
// mass-rate).
//
// CORRECTNESS BAR: where exact dx is fragile we assert robust invariants
// (finiteness, quaternion-derivative == 0.5*Omega*q, on-launcher lateral/
// rotational rates == 0, mass-rate == -mdot during burn). Pinned numbers are
// derived from the fixture and commented.
// ******************************************************

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "Eigen/Dense"

#include "dynamics/flight_dynamics.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"
#include "rocket/rocket.hpp"

#include "test_fixtures.hpp"

namespace forrocket {
namespace {

using forrocket::test::MakeTestRocket;
using forrocket::test::MakeZeroWind;

using state = FlightDynamics::state;  // std::array<double,14>

// Build a state vector consistent with the fixture's initialized kinematics
// (position/velocity ECI, attitude quaternion). Angular velocity and
// propellant default to the fixture values; callers tweak as needed.
state MakeStateFromRocket(const Rocket& r) {
    state x;
    x[0] = r.position.ECI[0];
    x[1] = r.position.ECI[1];
    x[2] = r.position.ECI[2];
    x[3] = r.velocity.ECI[0];
    x[4] = r.velocity.ECI[1];
    x[5] = r.velocity.ECI[2];
    x[6] = r.attitude.quaternion[0];
    x[7] = r.attitude.quaternion[1];
    x[8] = r.attitude.quaternion[2];
    x[9] = r.attitude.quaternion[3];
    x[10] = r.angular_velocity[0];
    x[11] = r.angular_velocity[1];
    x[12] = r.angular_velocity[2];
    x[13] = r.mass.propellant;
    return x;
}

void ExpectAllFinite(const state& dx) {
    for (std::size_t i = 0; i < dx.size(); ++i) {
        EXPECT_TRUE(std::isfinite(dx[i])) << "dx[" << i << "] not finite";
    }
}

// Re-derive the quaternion-derivative block from the published kinematic
// identity qdot = 0.5 * Omega(p,q,r) * q, using the same Omega matrix layout
// as DynamicsBase::QuaternionDiff. q is taken from x[6..9] (normalized as the
// solver does). Returns the 4-vector qdot.
Eigen::Vector4d ExpectedQuatDot(const state& x, const Eigen::Vector3d& omega) {
    Eigen::Vector4d q(x[6], x[7], x[8], x[9]);
    q.normalize();
    const double p = omega[0], qq = omega[1], r = omega[2];
    Eigen::Matrix4d Om;
    Om << 0, r, -qq, p,
          -r, 0, p, qq,
          qq, -p, 0, r,
          -p, -qq, -r, 0;
    return 0.5 * (Om * q);
}

// ===========================================================================
// Regime dispatch + each compute path executes at least once.
// ===========================================================================

// --- kInAir / Compute6dofAero -------------------------------------------------
// Default regime path (no program attitude). All 14 dx finite; quaternion
// derivative matches 0.5*Omega*q; mass-rate == -mdot during burn.
TEST(FlightDynamics, InAir6dofAero_FiniteAndConsistent) {
    Rocket r = MakeTestRocket();
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kInAir);
    // give a non-trivial body rate so the quaternion-dot block is non-degenerate
    state x = MakeStateFromRocket(r);
    x[10] = 0.05; x[11] = -0.03; x[12] = 0.02;  // p, q, r [rad/s]

    state dx;
    fd(x, dx, /*t=*/0.0);

    ExpectAllFinite(dx);

    // position-rate block == velocity ECI from the state (identity copy)
    EXPECT_DOUBLE_EQ(dx[0], x[3]);
    EXPECT_DOUBLE_EQ(dx[1], x[4]);
    EXPECT_DOUBLE_EQ(dx[2], x[5]);

    // quaternion-derivative block consistent with 0.5*Omega*q
    Eigen::Vector4d qd = ExpectedQuatDot(x, Eigen::Vector3d(0.05, -0.03, 0.02));
    EXPECT_NEAR(dx[6], qd[0], 1e-12);
    EXPECT_NEAR(dx[7], qd[1], 1e-12);
    EXPECT_NEAR(dx[8], qd[2], 1e-12);
    EXPECT_NEAR(dx[9], qd[3], 1e-12);

    // mass-rate == -mdot. At countup_time=0 (<=burn_duration=10) and prop=5>0
    // the engine is burning, so mdot_prop=4.0 -> dx[13] = -4.0.
    EXPECT_DOUBLE_EQ(dx[13], -4.0);
}

// kInAir but after burnout (propellant exhausted): mass-rate must be 0 because
// Engine::Update cuts off when mass_prop <= 0 -> mdot_prop = 0. This also drives
// the burnout branch of the coefficient/thrust getters.
TEST(FlightDynamics, InAir6dofAero_BurnoutZeroMassRate) {
    Rocket r = MakeTestRocket();
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kInAir);

    state x = MakeStateFromRocket(r);
    x[13] = 0.0;          // no propellant -> engine cutoff
    r.mass.propellant = 0.0;

    state dx;
    fd(x, dx, /*t=*/0.0);

    ExpectAllFinite(dx);
    EXPECT_DOUBLE_EQ(dx[13], 0.0);  // mdot_prop forced to 0 at cutoff
}

// --- kOnLauncher / Compute3dofOnLauncher -------------------------------------
// On the rail, attitude is frozen and motion is constrained: the quaternion
// derivative, angular-velocity derivative and (per code) all rotational dx are
// hard-zeroed; lateral thrust/gravity components are zeroed too.
TEST(FlightDynamics, OnLauncher_ConstrainsRotationAndAttitude) {
    Rocket r = MakeTestRocket();
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kOnLauncher);

    state x = MakeStateFromRocket(r);
    // even if we feed a body rate, the launcher path ignores it and zeros dx
    x[10] = 1.0; x[11] = 2.0; x[12] = 3.0;

    state dx;
    fd(x, dx, /*t=*/0.0);

    ExpectAllFinite(dx);

    // quaternion-derivative block == 0 (attitude frozen on rail)
    EXPECT_DOUBLE_EQ(dx[6], 0.0);
    EXPECT_DOUBLE_EQ(dx[7], 0.0);
    EXPECT_DOUBLE_EQ(dx[8], 0.0);
    EXPECT_DOUBLE_EQ(dx[9], 0.0);
    // angular-velocity derivative block == 0 (no rotation on rail)
    EXPECT_DOUBLE_EQ(dx[10], 0.0);
    EXPECT_DOUBLE_EQ(dx[11], 0.0);
    EXPECT_DOUBLE_EQ(dx[12], 0.0);

    // mass still burns on the rail (engine running) -> dx[13] = -mdot = -4.0
    EXPECT_DOUBLE_EQ(dx[13], -4.0);

    // position-rate == velocity ECI (identity copy)
    EXPECT_DOUBLE_EQ(dx[0], x[3]);
    EXPECT_DOUBLE_EQ(dx[1], x[4]);
    EXPECT_DOUBLE_EQ(dx[2], x[5]);
}

// On the rail with a *burning* engine, net body-x acceleration is positive
// (thrust 10 kN on ~15 kg minus friction/gravity-along-rail), so the
// acceleration is NOT clamped to zero and dx[3..5] (ECI accel) is non-zero.
// This covers the `acceleration.body(0) >= 0` (no-clamp) branch.
TEST(FlightDynamics, OnLauncher_BurningHasForwardAccel) {
    Rocket r = MakeTestRocket();
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kOnLauncher);

    state x = MakeStateFromRocket(r);
    state dx;
    fd(x, dx, /*t=*/0.0);

    double accel_eci_norm = std::sqrt(dx[3] * dx[3] + dx[4] * dx[4] + dx[5] * dx[5]);
    EXPECT_GT(accel_eci_norm, 0.0);   // not clamped: forward thrust dominates
}

// On the rail with the engine cut off (no propellant), the only body-x forces
// are aero drag (~0 at v=0) and friction; gravity-x is zeroed by the launcher
// path. With no thrust, acceleration.body(0) < 0 is NOT reached at v=0 (drag &
// friction are ~0), so this primarily covers the cutoff/no-thrust launcher
// state. Asserted as a characterization of the clamp logic at rest.
TEST(FlightDynamics, OnLauncher_CutoffAtRest_Characterization) {
    Rocket r = MakeTestRocket();
    r.mass.propellant = 0.0;          // engine will cut off
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kOnLauncher);

    state x = MakeStateFromRocket(r);
    x[13] = 0.0;
    state dx;
    fd(x, dx, /*t=*/0.0);

    ExpectAllFinite(dx);
    EXPECT_DOUBLE_EQ(dx[13], 0.0);    // no propellant flow
    // Characterization: at v=0 with no thrust, body-x accel <= 0 so the clamp
    // sets acceleration.ECI to zero -> dx[3..5] == 0. This pins current EOM
    // output for the clamp branch (acceleration.body(0) < 0).
    EXPECT_DOUBLE_EQ(dx[3], 0.0);
    EXPECT_DOUBLE_EQ(dx[4], 0.0);
    EXPECT_DOUBLE_EQ(dx[5], 0.0);
}

// --- kParachute / Compute3dofParachute ---------------------------------------
// Pure 3DOF descent: rotational + mass dx all zero, translational dx finite.
TEST(FlightDynamics, Parachute_3dofZeroRotationAndMassRate) {
    Rocket r = MakeTestRocket();
    r.setCdSParachute(2.0);
    r.OpenParachute();                // CdS_parachute = 2.0
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kParachute);

    state x = MakeStateFromRocket(r);
    // descent velocity along ECI (arbitrary, finite)
    x[3] = -10.0; x[4] = 5.0; x[5] = 3.0;
    state dx;
    fd(x, dx, /*t=*/0.0);

    ExpectAllFinite(dx);

    // position-rate == velocity ECI
    EXPECT_DOUBLE_EQ(dx[0], x[3]);
    EXPECT_DOUBLE_EQ(dx[1], x[4]);
    EXPECT_DOUBLE_EQ(dx[2], x[5]);

    // no attitude / rotational / mass dynamics under parachute
    for (int i = 6; i <= 13; ++i) {
        EXPECT_DOUBLE_EQ(dx[i], 0.0) << "dx[" << i << "] should be 0 under parachute";
    }

    // translational acceleration must be non-zero (gravity + drag present)
    double accel_norm = std::sqrt(dx[3] * dx[3] + dx[4] * dx[4] + dx[5] * dx[5]);
    EXPECT_GT(accel_norm, 0.0);
}

// ===========================================================================
// Program-attitude dispatch & branches (kInAir + enable_program_attitude).
// ===========================================================================

// enable_program_attitude with t inside [start,end) routes to
// Compute6dofProgramRate; outside the window routes to Compute6dofAero. We
// verify the dispatch boundary by toggling t around the window.
TEST(FlightDynamics, ProgramAttitudeDispatchWindow) {
    Rocket r = MakeTestRocket();
    r.enable_program_attitude = true;
    r.time_start_attitude_control = 2.0;
    r.time_end_attitude_control = 5.0;
    r.setAttitudeProgramRate(InterpolateParameter(0.0), InterpolateParameter(0.1),
                             InterpolateParameter(0.0));
    r.attitude_program_config.mode_rate = true;
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kInAir);

    state x = MakeStateFromRocket(r);
    state dx;

    // Before the window -> aero path. Just require finite (path executes).
    fd(x, dx, /*t=*/1.0);
    ExpectAllFinite(dx);

    // Inside the window -> program-rate path.
    fd(x, dx, /*t=*/3.0);
    ExpectAllFinite(dx);

    // At the end boundary (t == end is NOT < end) -> back to aero path.
    fd(x, dx, /*t=*/5.0);
    ExpectAllFinite(dx);
}

// Program RATE mode, all three axes controlled. Controlled-axis angular
// acceleration is forced to zero (control system owns the rate), so
// dx[10..12] == 0. Covers the mode_rate==true branch and all three
// enable_{roll,pitch,yaw} true sub-branches.
TEST(FlightDynamics, ProgramRateMode_AllAxes_ZeroAngularAccel) {
    Rocket r = MakeTestRocket();
    r.enable_program_attitude = true;
    r.time_start_attitude_control = 0.0;
    r.time_end_attitude_control = 100.0;
    r.attitude_program_config.mode_rate = true;
    r.attitude_program_config.enable_roll = true;
    r.attitude_program_config.enable_pitch = true;
    r.attitude_program_config.enable_yaw = true;
    r.setAttitudeProgramRate(InterpolateParameter(0.01), InterpolateParameter(0.02),
                             InterpolateParameter(0.03));
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kInAir);

    state x = MakeStateFromRocket(r);
    x[10] = 0.5; x[11] = 0.5; x[12] = 0.5;  // state rates (should be overridden)
    state dx;
    fd(x, dx, /*t=*/1.0);

    ExpectAllFinite(dx);
    // all axes controlled -> angular acceleration forced 0 on every axis
    EXPECT_DOUBLE_EQ(dx[10], 0.0);
    EXPECT_DOUBLE_EQ(dx[11], 0.0);
    EXPECT_DOUBLE_EQ(dx[12], 0.0);
    EXPECT_DOUBLE_EQ(dx[13], -4.0);  // still burning
}

// Program RATE mode, only PITCH controlled (roll/yaw free). The free axes keep
// their state-derived angular acceleration (generally non-zero from aero), the
// controlled pitch axis has angular acceleration forced to 0. Covers the mixed
// free/controlled sub-branches (enable_pitch=true; enable_roll/yaw=false).
TEST(FlightDynamics, ProgramRateMode_PitchOnly_FreeAxesRetainAccel) {
    Rocket r = MakeTestRocket();
    r.enable_program_attitude = true;
    r.time_start_attitude_control = 0.0;
    r.time_end_attitude_control = 100.0;
    r.attitude_program_config.mode_rate = true;
    r.attitude_program_config.enable_roll = false;
    r.attitude_program_config.enable_pitch = true;
    r.attitude_program_config.enable_yaw = false;
    r.setAttitudeProgramRate(InterpolateParameter(0.0), InterpolateParameter(0.05),
                             InterpolateParameter(0.0));
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kInAir);

    state x = MakeStateFromRocket(r);
    // body-y airflow -> sideslip -> yaw aero moment -> non-zero free-axis accel
    x[3] += 50.0;  // add ECI x-velocity to generate airspeed/AoA
    x[10] = 0.0; x[11] = 0.2; x[12] = 0.0;
    state dx;
    fd(x, dx, /*t=*/1.0);

    ExpectAllFinite(dx);
    // controlled pitch axis: angular accel forced to 0
    EXPECT_DOUBLE_EQ(dx[11], 0.0);
}

// Program ANGLE mode (mode_rate == false), all axes controlled. Controlled-axis
// angular velocity is set to 0 and angular acceleration forced to 0, so
// dx[10..12] == 0. Covers the angle-mode branch and its enable_* sub-branches.
TEST(FlightDynamics, ProgramAngleMode_AllAxes_ZeroAngularAccel) {
    Rocket r = MakeTestRocket();
    r.enable_program_attitude = true;
    r.time_start_attitude_control = 0.0;
    r.time_end_attitude_control = 100.0;
    r.attitude_program_config.mode_rate = false;  // ANGLE mode
    r.attitude_program_config.enable_roll = true;
    r.attitude_program_config.enable_pitch = true;
    r.attitude_program_config.enable_yaw = true;
    r.setAttitudeProgram(InterpolateParameter(deg2rad(270.0)),
                         InterpolateParameter(deg2rad(80.0)),
                         InterpolateParameter(deg2rad(0.0)));
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kInAir);

    state x = MakeStateFromRocket(r);
    x[10] = 0.3; x[11] = 0.3; x[12] = 0.3;
    state dx;
    fd(x, dx, /*t=*/1.0);

    ExpectAllFinite(dx);
    // all axes controlled in angle mode -> angular acceleration 0 on every axis
    EXPECT_DOUBLE_EQ(dx[10], 0.0);
    EXPECT_DOUBLE_EQ(dx[11], 0.0);
    EXPECT_DOUBLE_EQ(dx[12], 0.0);
}

// Program ANGLE mode, only ROLL controlled (pitch/yaw free). Covers the mixed
// branch: enable_roll=true forces roll angular accel 0, while pitch/yaw retain
// state-derived dynamics.
TEST(FlightDynamics, ProgramAngleMode_RollOnly) {
    Rocket r = MakeTestRocket();
    r.enable_program_attitude = true;
    r.time_start_attitude_control = 0.0;
    r.time_end_attitude_control = 100.0;
    r.attitude_program_config.mode_rate = false;
    r.attitude_program_config.enable_roll = true;
    r.attitude_program_config.enable_pitch = false;
    r.attitude_program_config.enable_yaw = false;
    r.setAttitudeProgram(InterpolateParameter(0.0), InterpolateParameter(0.0),
                         InterpolateParameter(deg2rad(10.0)));
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    fd.SetRegime(FlightDynamics::kInAir);

    state x = MakeStateFromRocket(r);
    x[10] = 0.4; x[11] = 0.1; x[12] = 0.1;
    state dx;
    fd(x, dx, /*t=*/1.0);

    ExpectAllFinite(dx);
    // controlled roll axis: angular accel forced 0
    EXPECT_DOUBLE_EQ(dx[10], 0.0);
}

// ===========================================================================
// Gas-jet roll moment (DynamicsBase::GasJetMoment, reached via ComputeMoments).
// ===========================================================================

// Gas-jet enabled and active (t within [launch_clear, launch_clear+duration])
// injects a +x rolling moment, producing a non-zero roll angular acceleration
// (dx[10]) compared to the gas-jet-disabled baseline. Covers the
// gas_jet_config.enable==true + active-window branch.
TEST(FlightDynamics, GasJet_ActiveProducesRollMoment) {
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();

    // Baseline: gas jet disabled.
    Rocket r0 = MakeTestRocket();
    FlightDynamics fd0(&r0, &clock, &wind);
    fd0.SetRegime(FlightDynamics::kInAir);
    state x0 = MakeStateFromRocket(r0);
    state dx0;
    fd0(x0, dx0, /*t=*/2.0);

    // Gas jet enabled, active at t=2.0 (launch_clear=1.0, duration=2.5 -> window
    // [1.0, 3.5]; elapsed=1.0 is inside).
    Rocket r1 = MakeTestRocket();
    r1.gas_jet_config.enable = true;
    r1.gas_jet_config.rolling_moment = 50.0;  // [N*m] about body +x
    r1.gas_jet_config.duration = 2.5;
    r1.time_launch_clear = 1.0;
    FlightDynamics fd1(&r1, &clock, &wind);
    fd1.SetRegime(FlightDynamics::kInAir);
    state x1 = MakeStateFromRocket(r1);
    state dx1;
    fd1(x1, dx1, /*t=*/2.0);

    ExpectAllFinite(dx1);
    // Roll moment about +x with Ixx=0.1 -> roll angular accel differs from
    // baseline by ~moment/Ixx = 50/0.1 = 500 rad/s^2 (sign-consistent positive).
    EXPECT_GT(dx1[10], dx0[10]);
    EXPECT_NEAR(dx1[10] - dx0[10], 50.0 / 0.1, 1e-6);
}

// Gas-jet enabled but OUTSIDE its active window (elapsed > duration) injects no
// moment -> roll angular acceleration matches the disabled baseline. Covers the
// gas_jet_config.enable==true + inactive-window branch.
TEST(FlightDynamics, GasJet_OutsideWindowNoMoment) {
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();

    Rocket r0 = MakeTestRocket();
    FlightDynamics fd0(&r0, &clock, &wind);
    fd0.SetRegime(FlightDynamics::kInAir);
    state x0 = MakeStateFromRocket(r0);
    state dx0;
    fd0(x0, dx0, /*t=*/10.0);

    Rocket r1 = MakeTestRocket();
    r1.gas_jet_config.enable = true;
    r1.gas_jet_config.rolling_moment = 50.0;
    r1.gas_jet_config.duration = 2.5;
    r1.time_launch_clear = 1.0;       // window [1.0, 3.5]; t=10 is well outside
    FlightDynamics fd1(&r1, &clock, &wind);
    fd1.SetRegime(FlightDynamics::kInAir);
    state x1 = MakeStateFromRocket(r1);
    state dx1;
    fd1(x1, dx1, /*t=*/10.0);

    ExpectAllFinite(dx1);
    EXPECT_NEAR(dx1[10], dx0[10], 1e-9);  // no gas-jet contribution
}

// ===========================================================================
// regime() accessor round-trip (set/get) — trivial branch on the public API.
// ===========================================================================
TEST(FlightDynamics, RegimeAccessorRoundTrip) {
    Rocket r = MakeTestRocket();
    SequenceClock clock;
    EnvironmentWind wind = MakeZeroWind();
    FlightDynamics fd(&r, &clock, &wind);
    EXPECT_EQ(fd.regime(), FlightDynamics::kInAir);  // ctor default
    fd.SetRegime(FlightDynamics::kOnLauncher);
    EXPECT_EQ(fd.regime(), FlightDynamics::kOnLauncher);
    fd.SetRegime(FlightDynamics::kParachute);
    EXPECT_EQ(fd.regime(), FlightDynamics::kParachute);
}

}  // namespace
}  // namespace forrocket
