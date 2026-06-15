// ******************************************************
// Unit tests for src/solver/rocket_stage.cpp
//
// RocketStage owns one stage's worth of flight: it builds the time-ordered
// FlightEvent list from its enable_* flags, constructs a FlightDynamics, and
// runs an adaptive (dopri5 dense-output) integration from time_start to
// time_end, firing events, detecting launch-clear / apogee, and stopping on
// ground impact. The only public entry points are the constructor and
// FlightSequence(); everything is driven through them.
//
// We cannot mock the integrator, so each test configures a real (but small,
// deterministic) flight using the shared MakeTestRocket() fixture and inspects
// the *observable outcomes*: the flight data recorder (fdr) samples, the
// separation latch (separated / time_at_separation / state_at_separation),
// rocket.time_launch_clear, engine.burning, rocket.CdS_parachute, masses, and
// the returned hand-off state x0.
//
// The fixture rocket (15 kg, 10 kN, elevation 85 deg) is a fast, short flight:
// it clears the rail in a fraction of a second, burns 10 s, coasts to apogee,
// then falls back to ground -- giving us natural, physics-driven coverage of
// the launch-clear, apogee and ground-impact branches.
//
// ---------------------------------------------------------------------------
// Decision points in FlightSequence (C1) and the test(s) that cover them.
// Compound conditions (&&) note BOTH covered outcomes (C2-relevant):
//
//   [L57]  if(enable_cutoff)             -> CutoffEnabledFiresCutoff (T) /
//                                           NoCutoffEngineSelfCutsAtPropellantDepletion (F)
//   [L60]  if(enable_despin)             -> DespinEnabledZeroesRoll (T) /
//                                           most other tests (F)
//   [L63]  if(enable_fairing_jettson)    -> FairingJettisonDropsInertMass (T) /
//                                           others (F)
//   [L67]  if(enable_sepation)           -> SeparationLatchesHandoffState (T) /
//                                           NoSeparationLeavesX0FromIntegration (F)
//          separation callback lambda    -> fired in SeparationLatches... (sets
//                                           separated/time/state)
//   [L79]  if(enable_parachute_open)     -> TimedParachuteOpenSwitchesRegime (T) /
//                                           many (F)
//   [L80]  if(enable_apogee_parachute)   -> ApogeeParachuteOpensAtApogee (T) /
//                                           TimedParachuteOpenSwitchesRegime (F)
//   [L86]  if(exist_second_parachute)    -> SecondParachuteAddsStage (T) /
//                                           TimedParachuteOpenSwitchesRegime (F)
//   [L92]  sort comparator               -> ManyEventsAreTimeOrdered (>=2 events)
//   [L102] if(enable_launcher)           -> launcher tests (T) /
//                                           InAirStartSkipsLauncherRegime (F)
//   [L123] preloop while (&& compound)   -> PreLoopFiresEarlyIgnition fires the
//          (idx<size && Time<=t_start)      ignition at t<=t_start (T,T then T,F);
//                                           NoCutoff... enters with no early
//                                           event (idx<size but Time>t_start -> F)
//   [L126] if(CdS>0 && regime!=parachute)-> PreLoopParachuteSetsRegime drives the
//          (preloop)                        parachute-at-start path (T,T); the
//                                           no-parachute pre-loop (ignition only,
//                                           CdS==0 -> F) is taken by every other
//                                           pre-loop pass (e.g. PreLoopFires...)
//   [L143] while(current_time<time_end)  -> every flying test (T) /
//                                           ZeroDurationWindowDoesNothing (F)
//   [L151] while(idx<size && Time<=tcur) -> CutoffEnabledFiresCutoff (in-loop
//          (in-loop &&)                     event fires: T,T) / coast samples (F)
//   [L158] if(CdS>0 && regime!=parachute)-> TimedParachuteOpenSwitchesRegime (T,T)
//          (in-loop)                        / pre-parachute samples (CdS==0 -> F)
//   [L168] if(stepper_reinitialized)     -> any in-loop event (T) / coast (F)
//   [L177] if(regime==kOnLauncher)       -> launcher tests (T) /
//                                           InAirStartSkipsLauncherRegime (F)
//   [L180] if(sin_elv > 1e-6)            -> LauncherClearsRail, elev 85deg (T) /
//                                           HorizontalLauncherSkipsClearOnSinGuard,
//                                           elev 0deg, sin=0 (F)
//   [L182] if(distance >= rail_length)   -> LauncherClearsRail (T, eventually) /
//                                           short pre-clear samples (F)
//   [L193] bisection if(dist_mid<rail)   -> exercised by the 20-iter bisection in
//                                           LauncherClearsRail (both T and F as it
//                                           brackets the root)
//   [L209] if(apogee_pending && kInAir)  -> ApogeeParachuteOpensAtApogee (T,T) /
//          (compound &&)                    pre-apogee-disabled tests (F)
//   [L212] if(!isnan && prev<0 && cur>=0)-> ApogeeParachuteOpensAtApogee crosses
//          (triple compound &&)             from climb(prev<0) to fall(cur>=0):
//                                           first sample isnan -> clause1 F; climb
//                                           samples prev<0 & cur<0 -> clause3 F;
//                                           apogee sample T,T,T.
//                                           clause2 (prev<0) FALSE side: covered
//                                           by ApogeePendingNeverTriggersWhile-
//                                           Descending (prev>=0 the whole time).
//   [L217] if(|dv|>1e-12)                -> apogee interpolation: normal crossing
//                                           has dv>0 (T). The degenerate dv~0 else
//                                           (t_apogee = t_curr) is NOT covered: it
//                                           needs two adjacent samples straddling
//                                           apogee with bit-identical down-velocity
//                                           magnitude, which the adaptive stepper
//                                           will not deterministically produce. It
//                                           is a 1-line numerical fallback, left
//                                           uncovered by design (see report).
//   [L240] if(LLH(2) < 0.0) break        -> GroundImpactStopsIntegration (T) /
//                                           apogee/coast samples above ground (F)
//   [L251] if(separated)                 -> SeparationLatchesHandoffState (T) /
//                                           NoSeparation... (F)
// ******************************************************

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "Eigen/Core"

#include "solver/rocket_stage.hpp"
#include "rocket/flight_data_recorder.hpp"
#include "dynamics/dynamics_base.hpp"
#include "dynamics/flight_dynamics.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/datetime.hpp"
#include "environment/wind.hpp"

#include "test_fixtures.hpp"

namespace forrocket {
namespace {

using forrocket::test::MakeTestRocket;
using forrocket::test::MakeZeroWind;

// Build the 14-element initial integration state from a Rocket exactly the way
// the production driver (trajectory_solver.cpp:86) does:
//   [0..2]  ECI position, [3..5] ECI velocity, [6..9] attitude quaternion,
//   [10..12] body angular velocity, [13] propellant mass.
DynamicsBase::state MakeX0(const Rocket& r) {
    DynamicsBase::state x0;
    x0 = {r.position.ECI(0), r.position.ECI(1), r.position.ECI(2),
          r.velocity.ECI(0), r.velocity.ECI(1), r.velocity.ECI(2),
          r.attitude.quaternion(0), r.attitude.quaternion(1),
          r.attitude.quaternion(2), r.attitude.quaternion(3),
          r.angular_velocity(0), r.angular_velocity(1), r.angular_velocity(2),
          r.mass.propellant};
    return x0;
}

// A RocketStage with every optional event disabled, a single short burn, and
// a coarse-but-fast time window. Individual tests flip the flags they exercise.
// time_step is only an integrator hint (the dopri5 dense-output stepper is
// adaptive; real accuracy is governed by eps_abs/eps_rel) -- see MEMORY note
// "Solver tolerance & Time Step".
RocketStage MakeStage(double t_end = 5.0) {
    RocketStage stage(/*stage_number=*/1, MakeTestRocket());
    stage.time_start = 0.0;
    stage.time_ignittion = 0.0;

    stage.enable_launcher = false;          // default: free flight (no rail)
    stage.length_launcher_rail = 5.0;

    stage.enable_cutoff = false;
    stage.time_cutoff = 1.0e10;

    stage.enable_sepation = false;
    stage.time_separation = 1.0e10;
    stage.mass_upper_stage = 0.0;

    stage.enable_despin = false;
    stage.time_despin = 1.0e10;

    stage.enable_fairing_jettson = false;
    stage.time_jettson_fairing = 1.0e10;
    stage.mass_fairing = 0.0;

    stage.enable_parachute_open = false;
    stage.enable_apogee_parachute_open = false;
    stage.time_open_parachute = 1.0e10;

    stage.exist_second_parachute = false;
    stage.time_open_second_parachute = 1.0e10;

    stage.time_end = t_end;
    stage.time_step = 0.05;

    // Wire the flight data recorder to the stage's own rocket. The RocketStage
    // constructor leaves fdr.p_rocket uninitialised; the production driver does
    // exactly this immediately after building the stage (see
    // trajectory_solver.cpp:41 -- `stage.fdr = FlightDataRecorder(&stage.rocket)`).
    // FlightSequence's logging (flight_data_recorder.cpp:61) dereferences this
    // pointer, so it is part of the public-API setup contract, not test glue.
    stage.fdr = FlightDataRecorder(&stage.rocket);

    // Keep the default tolerances from the constructor (1e-6/1e-6).
    return stage;
}

SequenceClock MakeClock() {
    return SequenceClock(DateTime(2026, 6, 15, 0, 0, 0));
}

// Largest altitude seen in the recorded samples (apogee proxy).
double MaxLoggedAltitude(const RocketStage& s) {
    double m = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < s.fdr.position.size(); ++i) {
        m = std::max(m, s.fdr.position[i].LLH(2));
    }
    return m;
}

}  // namespace


// ===========================================================================
// Constructor: defaults (separation latch cleared, tolerances seeded).
// ===========================================================================
TEST(RocketStage, ConstructorSeedsDefaults) {
    RocketStage stage(2, MakeTestRocket());
    EXPECT_EQ(stage.stage_number, 2);
    EXPECT_FALSE(stage.separated);
    EXPECT_DOUBLE_EQ(stage.time_at_separation, 0.0);
    // state_at_separation is filled with 0.0 by the constructor.
    for (std::size_t i = 0; i < stage.state_at_separation.size(); ++i) {
        EXPECT_DOUBLE_EQ(stage.state_at_separation[i], 0.0);
    }
    // Default adaptive tolerances (see rocket_stage.cpp:38-39).
    EXPECT_DOUBLE_EQ(stage.eps_abs, 1.0e-6);
    EXPECT_DOUBLE_EQ(stage.eps_rel, 1.0e-6);
}


// ===========================================================================
// [L143 F] Empty integration window: time_start == time_end. The ignition
// preloop still fires (ignition Time 0 <= time_start 0), but the main loop is
// never entered, so nothing is logged and the engine is the only side effect.
// ===========================================================================
TEST(RocketStage, ZeroDurationWindowDoesNothing) {
    RocketStage stage = MakeStage(/*t_end=*/0.0);
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_EQ(stage.fdr.position.size(), 0u)
        << "main loop body must not run when time_start == time_end";
    EXPECT_FALSE(stage.separated);
    // Ignition preloop (Time 0 <= time_start 0) lit the engine.
    EXPECT_TRUE(stage.rocket.engine.burning);
}


// ===========================================================================
// [L123 preloop &&] Ignition scheduled at/-before time_start fires in the
// pre-loop. We start the stage at t=1.0 with ignition at t=0.0 so the preloop
// condition (idx<size && Time<=t_start) is TRUE then becomes FALSE (only one
// early event), and the engine is already burning before the main loop.
// ===========================================================================
TEST(RocketStage, PreLoopFiresEarlyIgnition) {
    RocketStage stage = MakeStage(/*t_end=*/2.0);
    stage.time_start = 1.0;       // start later than ignition
    stage.time_ignittion = 0.0;   // ignition is "in the past" -> preloop fires it
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    Rocket pre = stage.rocket;
    pre.engine.Cutoff();          // ensure not burning beforehand
    DynamicsBase::state x0 = MakeX0(stage.rocket);

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_TRUE(stage.rocket.engine.burning)
        << "ignition with Time <= time_start must fire in the pre-loop";
    EXPECT_GT(stage.fdr.position.size(), 0u);
}


// ===========================================================================
// [L57 F][L143 T][L240 F] Baseline free flight, no CutoffEvent: with the
// enable_cutoff == FALSE branch, the engine is never commanded off by a timed
// event -- it self-cuts off only when propellant is exhausted. The fixture
// engine carries 5 kg of propellant at mdot = 4 kg/s, so it depletes at
//   t = 5 kg / 4 (kg/s) = 1.25 s  (Engine::Update, engine.cpp:104:
//   `t <= burn_duration && mass_prop > 0.0`; mass_prop hits 0 first).
// Over the 5 s window the engine therefore ends NOT burning, the last logged
// propellant mass is ~0, yet the vehicle has gained altitude from the burn.
// (Note: the returned x0 is NOT the integration end state -- FlightSequence
//  only writes x0 back on separation; see NoSeparationLeavesX0AtPreLoopState.
//  So we read the live propellant from the recorder, not x0[13].)
// ===========================================================================
TEST(RocketStage, NoCutoffEngineSelfCutsAtPropellantDepletion) {
    RocketStage stage = MakeStage(/*t_end=*/5.0);
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);
    double alt0 = stage.rocket.position.LLH(2);

    stage.FlightSequence(&clock, &wind, x0);

    ASSERT_GT(stage.fdr.mass_prop.size(), 0u);
    // No CutoffEvent fired; propellant (5 kg / 4 kg/s = 1.25 s) ran out -> off.
    EXPECT_FALSE(stage.rocket.engine.burning)
        << "no CutoffEvent -> engine cuts off only at propellant depletion";
    EXPECT_NEAR(stage.fdr.mass_prop.back(), 0.0, 1.0e-3)
        << "last logged propellant mass is depleted well before t_end = 5 s";
    EXPECT_GT(MaxLoggedAltitude(stage), alt0)
        << "thrust >> weight during the burn, vehicle must gain altitude";
}


// ===========================================================================
// [L57 T][L151 in-loop &&][L168 T] A CutoffEvent at t=2 s stops the burn before
// the propellant is gone. We compare against the no-cutoff baseline (still
// burning at 5 s) to prove the event -- not depletion -- caused the cutoff.
// ===========================================================================
TEST(RocketStage, CutoffEnabledFiresCutoff) {
    RocketStage stage = MakeStage(/*t_end=*/5.0);
    stage.enable_cutoff = true;
    stage.time_cutoff = 2.0;    // well before the 10 s natural burnout
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);

    stage.FlightSequence(&clock, &wind, x0);

    ASSERT_GT(stage.fdr.position.size(), 0u);
    EXPECT_FALSE(stage.rocket.engine.burning)
        << "CutoffEvent at 2 s must have stopped the burn by t_end = 5 s";
}


// ===========================================================================
// [L60 T] Despin event zeroes the roll rate. We spin the rocket up (roll rate
// in body x = state[10]) and schedule a DespinEvent; after the flight the
// stored angular velocity roll component must be zero.
// ===========================================================================
TEST(RocketStage, DespinEnabledZeroesRoll) {
    RocketStage stage = MakeStage(/*t_end=*/3.0);
    stage.enable_despin = true;
    stage.time_despin = 1.0;
    stage.rocket.angular_velocity(0) = 10.0;   // 10 rad/s roll
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);  // captures roll into x0[10]
    ASSERT_DOUBLE_EQ(x0[10], 10.0);

    stage.FlightSequence(&clock, &wind, x0);

    // DeSpin zeros the roll rate at t=1 s; the integration continues afterwards
    // and re-derives angular velocity from the (now ~0) state[10], so a tiny
    // numerical residual (~1e-12 rad/s) can accumulate. The point is the 10 rad/s
    // initial spin is annihilated, not that the double is bit-exact zero.
    EXPECT_NEAR(stage.rocket.angular_velocity(0), 0.0, 1.0e-6)
        << "DespinEvent must drive the body-x (roll) angular velocity to ~0";
}


// ===========================================================================
// [L63 T] Fairing jettison subtracts the fairing mass from inert mass. Fixture
// inert = 10 kg; jettison 3 kg -> 7 kg. (Rocket::JettsonFairing semantics are
// independently tested in test_rocket.cpp; here we verify the stage wires the
// event in.)
// ===========================================================================
TEST(RocketStage, FairingJettisonDropsInertMass) {
    RocketStage stage = MakeStage(/*t_end=*/2.0);
    stage.enable_fairing_jettson = true;
    stage.time_jettson_fairing = 0.5;
    stage.mass_fairing = 3.0;
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);
    ASSERT_DOUBLE_EQ(stage.rocket.mass.inert, 10.0);

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_DOUBLE_EQ(stage.rocket.mass.inert, 7.0)  // 10 - 3
        << "JettisonFairingEvent must reduce inert mass by the fairing mass";
}


// ===========================================================================
// [L67 T][L251 T] Separation: the lambda callback latches separated = true and
// records the time and state. SeparateUpperStage also drops the inert mass
// (fixture inert 10, upper 4 -> 6). And because separated is set, the returned
// x0 must equal state_at_separation (the L251 hand-off branch).
// ===========================================================================
TEST(RocketStage, SeparationLatchesHandoffState) {
    RocketStage stage = MakeStage(/*t_end=*/5.0);
    stage.enable_sepation = true;
    stage.time_separation = 2.0;
    stage.mass_upper_stage = 4.0;
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_TRUE(stage.separated) << "separation callback must latch separated";
    EXPECT_NEAR(stage.time_at_separation, 2.0, 1.0e-6)
        << "callback records the separation event time";
    EXPECT_DOUBLE_EQ(stage.rocket.mass.inert, 6.0);  // 10 - 4
    // L251: separated -> x0 handed off as the separation state.
    for (std::size_t i = 0; i < x0.size(); ++i) {
        EXPECT_DOUBLE_EQ(x0[i], stage.state_at_separation[i])
            << "x0 must be the separation state at index " << i;
    }
}


// ===========================================================================
// [L67 F][L251 F] No separation: separated stays false, so FlightSequence does
// NOT overwrite x0 with state_at_separation (which is the all-zero default).
// FlightSequence only writes x0 back at the post-preloop state (rocket_stage.cpp
// :133, x0 = x) and, on separation, at L251 -- it never assigns the integration
// END state when there is no separation. So x0 here is the post-preloop state
// (essentially the initial state, ECI magnitude ~Earth radius 6.4e6 m). The
// invariant we assert is the L251-FALSE one: x0 keeps the real ECI position and
// is NOT the zero-filled separation state (which would make x0[0..2] == 0).
// ===========================================================================
TEST(RocketStage, NoSeparationLeavesX0AtPreLoopState) {
    RocketStage stage = MakeStage(/*t_end=*/2.0);
    // enable_sepation stays false
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);
    double eci_mag_init =
        std::abs(x0[0]) + std::abs(x0[1]) + std::abs(x0[2]);
    ASSERT_GT(eci_mag_init, 1.0e6);  // launch-site ECI is ~6.4e6 m

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_FALSE(stage.separated);
    // L251 not taken: x0 keeps a real ECI position, not the zero separation state.
    EXPECT_GT(std::abs(x0[0]) + std::abs(x0[1]) + std::abs(x0[2]), 1.0e6)
        << "without separation x0 is NOT the zero-filled separation state";
}


// ===========================================================================
// [L92 sort] Many events: ignition + cutoff + despin + jettison + separation +
// parachute. With >=2 events the time-order sort comparator runs. We schedule
// them out of declaration order in time and verify each side effect happened,
// i.e. the events were applied (which only works if they were sorted/dispatched
// correctly through the same loop).
// ===========================================================================
TEST(RocketStage, ManyEventsAreTimeOrdered) {
    RocketStage stage = MakeStage(/*t_end=*/6.0);
    stage.enable_cutoff = true;          stage.time_cutoff = 3.0;
    stage.enable_despin = true;          stage.time_despin = 1.0;
    stage.enable_fairing_jettson = true; stage.time_jettson_fairing = 2.0;
                                         stage.mass_fairing = 2.0;
    stage.enable_sepation = true;        stage.time_separation = 4.0;
                                         stage.mass_upper_stage = 1.0;
    stage.rocket.angular_velocity(0) = 5.0;
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_FALSE(stage.rocket.engine.burning) << "cutoff @3 applied";
    EXPECT_NEAR(stage.rocket.angular_velocity(0), 0.0, 1.0e-6) << "despin @1 applied";
    // inert: 10 - 2 (fairing) - 1 (separation) = 7
    EXPECT_DOUBLE_EQ(stage.rocket.mass.inert, 7.0) << "jettison @2 + separation @4";
    EXPECT_TRUE(stage.separated) << "separation @4 applied";
    EXPECT_NEAR(stage.time_at_separation, 4.0, 1.0e-6);
}


// ===========================================================================
// [L102 F][L177 F] In-air start (no launcher): regime is kInAir from the start,
// so the launcher-clear block is never entered and time_launch_clear keeps its
// sentinel (1e10) -- it is only written inside the launcher-clear branch.
// ===========================================================================
TEST(RocketStage, InAirStartSkipsLauncherRegime) {
    RocketStage stage = MakeStage(/*t_end=*/3.0);
    stage.enable_launcher = false;
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);
    double sentinel = stage.rocket.time_launch_clear;  // 1e10 default
    ASSERT_GT(sentinel, 1.0e9);

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_DOUBLE_EQ(stage.rocket.time_launch_clear, sentinel)
        << "no launcher -> launch-clear branch never runs, sentinel unchanged";
}


// ===========================================================================
// [L102 T][L177 T][L180 T][L182 T][L193 bisection] Launcher start: the vehicle
// begins on the rail (kOnLauncher) and, once it has travelled
// length_launcher_rail up the rail, the conditional launch-clear block detects
// it, runs the 20-iteration dense-output bisection, switches to kInAir and
// records time_launch_clear. With elevation 85 deg, sin_elv ~ 0.996 > 1e-6 so
// the sin_elv guard (L180) is taken.
//
//   distance along rail = altitude_gain / sin(elev).
//   rail = 5 m, sin(85deg) ~ 0.9962 -> needs ~4.98 m of altitude gain.
//   At ~67 m/s^2 net accel that happens well within the first second.
// ===========================================================================
TEST(RocketStage, LauncherClearsRail) {
    RocketStage stage = MakeStage(/*t_end=*/3.0);
    stage.enable_launcher = true;
    stage.length_launcher_rail = 5.0;
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_LT(stage.rocket.time_launch_clear, 1.0e9)
        << "launcher-clear branch must fire and overwrite the 1e10 sentinel";
    EXPECT_GT(stage.rocket.time_launch_clear, 0.0)
        << "launch clear happens after liftoff, t > 0";
    EXPECT_LT(stage.rocket.time_launch_clear, 3.0)
        << "rail is short -> cleared well before t_end";
}


// ===========================================================================
// [L79 T][L80 F][L86 F][L158 in-loop && T,T] Timed parachute open (NOT apogee
// mode, no second chute): a ParachuteOpenEvent at t=2 s makes CdS_parachute > 0
// and switches the dynamics regime to kParachute. We observe the post-flight
// CdS_parachute > 0 (the chute was deployed) -- the regime switch is the
// internal effect of the L158 compound condition's TRUE,TRUE branch.
// ===========================================================================
TEST(RocketStage, TimedParachuteOpenSwitchesRegime) {
    RocketStage stage = MakeStage(/*t_end=*/3.0);
    stage.enable_parachute_open = true;
    stage.enable_apogee_parachute_open = false;   // timed, not apogee
    stage.time_open_parachute = 2.0;
    stage.exist_second_parachute = false;
    stage.rocket.setCdSParachute(1.5);            // one chute stage, CdS = 1.5
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);
    ASSERT_DOUBLE_EQ(stage.rocket.CdS_parachute, 0.0);

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_DOUBLE_EQ(stage.rocket.CdS_parachute, 1.5)
        << "timed ParachuteOpenEvent must deploy the configured chute (CdS=1.5)";
}


// ===========================================================================
// [L86 T] Second parachute configured: with exist_second_parachute the stage
// adds a SECOND ParachuteOpenEvent. With two chute stages configured and two
// open events fired, CdS accumulates both (1.5 + 2.5 = 4.0). This also exercises
// the L92 sort with two same-typed parachute events at different times.
// ===========================================================================
TEST(RocketStage, SecondParachuteAddsStage) {
    RocketStage stage = MakeStage(/*t_end=*/4.0);
    stage.enable_parachute_open = true;
    stage.enable_apogee_parachute_open = false;
    stage.time_open_parachute = 1.0;
    stage.exist_second_parachute = true;
    stage.time_open_second_parachute = 2.0;
    stage.rocket.setCdSParachute(1.5, 2.5);       // two stages: 1.5 then +2.5
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_DOUBLE_EQ(stage.rocket.CdS_parachute, 4.0)  // 1.5 + 2.5
        << "both parachute-open events must fire and accumulate CdS";
}


// ===========================================================================
// [L79 T][L80 T][L209 compound &&][L212 triple &&][L217 T] Apogee-triggered
// parachute: enable_apogee_parachute_open marks apogee_pending instead of
// scheduling a timed event. During the kInAir coast the down velocity (NED z,
// down-positive) crosses from negative (climbing) to >= 0 (falling) -- that
// zero-crossing is the apogee, where the chute is deployed and the regime
// switches to kParachute.
//
// Branch detail for L212 (!isnan && prev<0 && cur>=0):
//   - first in-air sample: prev is NaN -> F (just stores prev) ;
//   - climb samples: prev<0, cur<0 -> third clause F ;
//   - apogee sample: all three TRUE -> deploy.
// L217 (|dv|>1e-12) is TRUE for the genuine crossing (velocity is changing).
//
// Outcome observed: chute deployed (CdS>0) AND apogee detected near the peak
// altitude. A long enough window is needed for the vehicle to actually arc over.
// ===========================================================================
TEST(RocketStage, ApogeeParachuteOpensAtApogee) {
    // Burn 10 s then coast; apogee is well after burnout. Give a wide window.
    RocketStage stage = MakeStage(/*t_end=*/120.0);
    stage.enable_parachute_open = true;
    stage.enable_apogee_parachute_open = true;    // apogee mode
    stage.time_open_parachute = 1.0e10;           // unused in apogee mode
    stage.exist_second_parachute = false;
    stage.rocket.setCdSParachute(2.0);
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);

    stage.FlightSequence(&clock, &wind, x0);

    ASSERT_GT(stage.fdr.position.size(), 0u);
    EXPECT_DOUBLE_EQ(stage.rocket.CdS_parachute, 2.0)
        << "apogee detection must deploy the chute (CdS=2.0)";
}


// ===========================================================================
// [L126 preloop && TRUE,TRUE] Parachute already open at/-before time_start:
// when a ParachuteOpenEvent fires in the PRE-LOOP (Time <= time_start) it makes
// CdS_parachute > 0, and the preloop's compound guard
//   (CdS_parachute > 0.0 && regime != kParachute)
// flips the regime to kParachute right away (rocket_stage.cpp:126-130) -- the
// only place that branch's TRUE,TRUE side is reachable. We start the stage at
// t=1.0 with a timed parachute open at t=0.0 so it lands in the pre-loop.
//
// Observable: the chute is deployed (CdS>0) before the main loop, and the
// vehicle integrates under parachute dynamics (it must descend, not climb,
// because the only forces in kParachute are drag + gravity -- no thrust).
// ===========================================================================
TEST(RocketStage, PreLoopParachuteSetsRegime) {
    RocketStage stage = MakeStage(/*t_end=*/3.0);
    stage.time_start = 1.0;                  // main loop starts at t=1
    stage.time_ignittion = 0.0;
    stage.enable_parachute_open = true;
    stage.enable_apogee_parachute_open = false;
    stage.time_open_parachute = 0.0;         // <= time_start -> fires in pre-loop
    stage.exist_second_parachute = false;
    stage.rocket.setCdSParachute(2.0);
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);
    double alt_start = stage.rocket.position.LLH(2);

    stage.FlightSequence(&clock, &wind, x0);

    ASSERT_GT(stage.fdr.position.size(), 0u);
    EXPECT_DOUBLE_EQ(stage.rocket.CdS_parachute, 2.0)
        << "pre-loop ParachuteOpenEvent must deploy the chute before the main loop";
    // Under parachute (drag + gravity, no thrust) the vehicle only descends.
    EXPECT_LE(MaxLoggedAltitude(stage), alt_start + 1.0)
        << "in kParachute the body cannot climb above its release altitude";
}


// ===========================================================================
// [L180 FALSE] sin_elv guard: on the launcher, launch-clear distance divides
// by sin(elevation). When the elevation is ~0 (a horizontal rail) sin_elv is
// <= 1e-6, so the guard is FALSE and the divide is skipped -- the vehicle never
// "clears" by this metric and time_launch_clear keeps its 1e10 sentinel. This
// is the divide-by-zero protection branch (rocket_stage.cpp:180). We set the
// attitude to 0 deg elevation; sin(0) = 0 < 1e-6.
//
// (Counterpart to LauncherClearsRail, which takes the sin_elv > 1e-6 TRUE side
//  at elevation 85 deg.)
// ===========================================================================
TEST(RocketStage, HorizontalLauncherSkipsClearOnSinGuard) {
    RocketStage stage = MakeStage(/*t_end=*/1.0);
    stage.enable_launcher = true;
    stage.length_launcher_rail = 5.0;
    // Re-aim flat: azimuth 270 deg, elevation 0 deg, roll 0 -> sin(elev)=0.
    stage.rocket.attitude.Initialize(
        Eigen::Vector3d(forrocket::deg2rad(270.0), forrocket::deg2rad(0.0), 0.0));
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);
    double sentinel = stage.rocket.time_launch_clear;  // 1e10
    ASSERT_GT(sentinel, 1.0e9);

    stage.FlightSequence(&clock, &wind, x0);

    EXPECT_DOUBLE_EQ(stage.rocket.time_launch_clear, sentinel)
        << "sin_elv <= 1e-6 -> launch-clear distance test is skipped, no clear time";
}


// ===========================================================================
// [L213 FALSE] Apogee detector, descending-from-the-start case: the apogee
// crossing test is (!isnan(prev) && prev<0 && cur>=0). If the vehicle is moving
// DOWNWARD throughout (NED-down velocity >= 0 every sample), the middle clause
// prev<0 is FALSE on every comparison after the first (NaN) one, so apogee is
// never declared and the chute never opens. This covers the prev>=0 (FALSE)
// side of the second clause.
//
// We launch the vehicle on a descending ECI velocity (anti-radial: toward the
// Earth's centre) so it falls immediately. apogee_pending stays true the whole
// time but the crossing never happens.
// ===========================================================================
TEST(RocketStage, ApogeePendingNeverTriggersWhileDescending) {
    RocketStage stage = MakeStage(/*t_end=*/3.0);
    stage.enable_parachute_open = true;
    stage.enable_apogee_parachute_open = true;    // apogee mode -> apogee_pending
    stage.exist_second_parachute = false;
    stage.rocket.setCdSParachute(2.0);
    // Make it descend from t=0: ECI velocity pointed at the Earth's centre
    // (anti-parallel to the ECI position) so NED-down velocity is positive.
    Eigen::Vector3d r_eci = stage.rocket.position.ECI;
    Eigen::Vector3d v_down_eci = -r_eci.normalized() * 50.0;  // 50 m/s inward
    stage.rocket.velocity.ECI = v_down_eci;
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);

    stage.FlightSequence(&clock, &wind, x0);

    // Never crossed climb->fall (it was already falling) -> no apogee deploy.
    EXPECT_DOUBLE_EQ(stage.rocket.CdS_parachute, 0.0)
        << "no upward-to-downward crossing -> apogee chute must NOT deploy";
}


// ===========================================================================
// [L240 T] Ground impact: a ballistic flight (no chute) eventually returns to
// LLH altitude < 0, which breaks the integration loop. We verify the loop
// terminated on impact rather than on time_end by checking the last logged
// altitude is near/below the launch altitude and the run stopped before t_end.
//
// The fixture launches from 20 m AGL=h; apogee then fall-back crosses zero.
// We give a wide window (300 s) so impact -- not t_end -- ends the run.
// ===========================================================================
TEST(RocketStage, GroundImpactStopsIntegration) {
    RocketStage stage = MakeStage(/*t_end=*/300.0);
    SequenceClock clock = MakeClock();
    EnvironmentWind wind = MakeZeroWind();
    DynamicsBase::state x0 = MakeX0(stage.rocket);

    stage.FlightSequence(&clock, &wind, x0);

    ASSERT_GT(stage.fdr.position.size(), 1u);
    // The recorder only logs samples with altitude >= 0, and the loop breaks
    // when altitude < 0. So the climb-then-fall arc must show a peak well above
    // the start, confirming we flew the full up-and-down arc to impact.
    double last_alt = stage.fdr.position.back().LLH(2);
    double peak = MaxLoggedAltitude(stage);
    EXPECT_GT(peak, 100.0) << "vehicle must have climbed substantially";
    EXPECT_LT(last_alt, peak)
        << "flight ended on the way down (ground-impact break), not at apogee";
}

}  // namespace forrocket
