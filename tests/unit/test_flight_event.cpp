// ******************************************************
// Unit tests for src/solver/flight_event.cpp
//   - FlightEvent hierarchy: Ignition / Cutoff / Despin / JettisonFairing /
//     Separation / ParachuteOpen.
//
// What is in flight_event.cpp: the constructors that take a clock pointer,
// and the Apply(Rocket&, state&) overrides. Apply() forwards to the matching
// Rocket SOE handler (and, for Separation, optionally fires a callback). The
// Time() accessors are trivial inline getters in the header.
//
// The Apply() methods are testable WITHOUT a heavy fixture: a default-
// constructed Rocket plus a SequenceClock is enough, because the SOE handlers
// they call only flip flags / adjust scalar mass / zero an angular rate. We
// therefore observe the *side effects* on the Rocket and on the integration
// state vector x.
//
// Decision points exercised (C1 coverage):
//   IgnitionEvent::Apply        -> engine.burning becomes true; clock seeded.
//   CutoffEvent::Apply          -> engine.burning becomes false.
//   DespinEvent::Apply          -> roll rate (x[10]) zeroed; spin state zeroed.
//   JettisonFairingEvent::Apply -> mass.inert reduced by fairing mass.
//   SeparationEvent::Apply      -> mass.inert reduced; BOTH callback branches:
//                                    (a) if(on_separate_) true  -> callback fires
//                                    (b) if(on_separate_) false -> no-op (null cb)
//   ParachuteOpenEvent::Apply   -> OpenParachute() invoked (CdS handling).
//   Time() getters on each event return the stored time.
// ******************************************************

#include <gtest/gtest.h>

#include "solver/flight_event.hpp"
#include "rocket/rocket.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/datetime.hpp"
#include "dynamics/dynamics_base.hpp"

using forrocket::Rocket;
using forrocket::SequenceClock;
using forrocket::DateTime;
using forrocket::DynamicsBase;
using forrocket::IgnitionEvent;
using forrocket::CutoffEvent;
using forrocket::DespinEvent;
using forrocket::JettisonFairingEvent;
using forrocket::SeparationEvent;
using forrocket::ParachuteOpenEvent;

namespace {

// A zeroed 14-element integration state (matches DynamicsBase::state layout).
DynamicsBase::state ZeroState() {
    DynamicsBase::state x;
    x.fill(0.0);
    return x;
}

}  // namespace


// ---------------------------------------------------------------------------
// Time() accessors: each event returns exactly the time it was constructed
// with. These are the trivial getters in flight_event.hpp.
// ---------------------------------------------------------------------------
TEST(FlightEvent, TimeAccessorsReturnConstructorTime) {
    DateTime utc(2026, 6, 15, 0, 0, 0);
    SequenceClock clock(utc);

    IgnitionEvent ig(3.5, &clock);
    CutoffEvent co(12.0);
    DespinEvent ds(20.0);
    JettisonFairingEvent jf(45.0, 2.0);
    SeparationEvent sep(90.0, 50.0, SeparationEvent::Callback());
    ParachuteOpenEvent po(120.0);

    EXPECT_DOUBLE_EQ(ig.Time(), 3.5);
    EXPECT_DOUBLE_EQ(co.Time(), 12.0);
    EXPECT_DOUBLE_EQ(ds.Time(), 20.0);
    EXPECT_DOUBLE_EQ(jf.Time(), 45.0);
    EXPECT_DOUBLE_EQ(sep.Time(), 90.0);
    EXPECT_DOUBLE_EQ(po.Time(), 120.0);
}


// ---------------------------------------------------------------------------
// IgnitionEvent::Apply -> Rocket::IgnitionEngine -> engine.Ignittion()
// sets engine.burning = true. Also reseeds rocket.burn_clock from the master
// clock's UTC_date_init and the event time.
// ---------------------------------------------------------------------------
TEST(FlightEvent, IgnitionEventStartsBurning) {
    DateTime utc(2026, 6, 15, 1, 2, 3);
    SequenceClock clock(utc);
    IgnitionEvent ig(2.0, &clock);

    Rocket rocket;
    rocket.engine.burning = false;  // explicit precondition
    DynamicsBase::state x = ZeroState();

    ig.Apply(rocket, x);

    EXPECT_TRUE(rocket.engine.burning) << "ignition must start the burn";
    // burn_clock was reseeded from the master clock's init date (same year).
    EXPECT_EQ(rocket.burn_clock.UTC_date_init.year, 2026u);
}


// ---------------------------------------------------------------------------
// CutoffEvent::Apply -> Rocket::CutoffEngine -> engine.Cutoff()
// sets engine.burning = false.
// ---------------------------------------------------------------------------
TEST(FlightEvent, CutoffEventStopsBurning) {
    CutoffEvent co(10.0);

    Rocket rocket;
    rocket.engine.burning = true;  // precondition: engine is lit
    DynamicsBase::state x = ZeroState();

    co.Apply(rocket, x);

    EXPECT_FALSE(rocket.engine.burning) << "cutoff must stop the burn";
}


// ---------------------------------------------------------------------------
// DespinEvent::Apply -> Rocket::DeSpin() zeroes body-x angular velocity and
// acceleration, AND the override itself zeroes the roll rate stored in the
// integration state x[10] (so the integrator stays continuous).
// ---------------------------------------------------------------------------
TEST(FlightEvent, DespinEventZeroesRollRate) {
    DespinEvent ds(15.0);

    Rocket rocket;
    rocket.angular_velocity[0] = 5.0;      // some roll rate, body x
    rocket.angular_acceleration[0] = 1.0;  // some roll accel, body x

    DynamicsBase::state x = ZeroState();
    x[10] = 7.0;  // pre-existing roll rate in the state vector

    ds.Apply(rocket, x);

    EXPECT_DOUBLE_EQ(x[10], 0.0) << "state roll rate must be zeroed";
    EXPECT_DOUBLE_EQ(rocket.angular_velocity[0], 0.0);
    EXPECT_DOUBLE_EQ(rocket.angular_acceleration[0], 0.0);
}


// ---------------------------------------------------------------------------
// JettisonFairingEvent::Apply -> Rocket::JettsonFairing(mass) subtracts the
// fairing mass from inert mass.
//   inert = 100, fairing = 30  => inert = 70.  (100 - 30 = 70)
// ---------------------------------------------------------------------------
TEST(FlightEvent, JettisonFairingReducesInertMass) {
    JettisonFairingEvent jf(50.0, 30.0);  // time, mass_fairing

    Rocket rocket;
    rocket.mass.inert = 100.0;
    rocket.mass.propellant = 0.0;
    DynamicsBase::state x = ZeroState();

    jf.Apply(rocket, x);

    EXPECT_DOUBLE_EQ(rocket.mass.inert, 70.0);  // 100 - 30
}


// ---------------------------------------------------------------------------
// SeparationEvent::Apply, branch (a): a non-null callback IS provided ->
// if(on_separate_) is TRUE -> the callback fires with (time, x).
// Also: Rocket::SeparateUpperStage subtracts the upper-stage mass.
//   inert = 200, upper = 80 => inert = 120.  (200 - 80 = 120)
// ---------------------------------------------------------------------------
TEST(FlightEvent, SeparationFiresCallbackWhenProvided) {
    bool callback_called = false;
    double seen_time = -1.0;

    SeparationEvent::Callback cb =
        [&](double t, const DynamicsBase::state& /*state*/) {
            callback_called = true;
            seen_time = t;
        };

    SeparationEvent sep(90.0, 80.0, cb);  // time, mass_upper_stage, callback

    Rocket rocket;
    rocket.mass.inert = 200.0;
    rocket.mass.propellant = 0.0;
    DynamicsBase::state x = ZeroState();

    sep.Apply(rocket, x);

    EXPECT_TRUE(callback_called) << "on_separate_ truthy branch must fire";
    EXPECT_DOUBLE_EQ(seen_time, 90.0) << "callback receives the event time";
    EXPECT_DOUBLE_EQ(rocket.mass.inert, 120.0);  // 200 - 80
}


// ---------------------------------------------------------------------------
// SeparationEvent::Apply, branch (b): a NULL callback is provided ->
// if(on_separate_) is FALSE -> no callback fired, but mass still drops.
//   inert = 200, upper = 80 => inert = 120.
// ---------------------------------------------------------------------------
TEST(FlightEvent, SeparationSkipsNullCallback) {
    SeparationEvent sep(90.0, 80.0, SeparationEvent::Callback());  // null cb

    Rocket rocket;
    rocket.mass.inert = 200.0;
    rocket.mass.propellant = 0.0;
    DynamicsBase::state x = ZeroState();

    // Must not throw / crash when the callback is empty.
    ASSERT_NO_FATAL_FAILURE(sep.Apply(rocket, x));

    EXPECT_DOUBLE_EQ(rocket.mass.inert, 120.0);  // 200 - 80
}


// ---------------------------------------------------------------------------
// SeparateUpperStage / JettsonFairing floor: if the subtraction would drive
// inert mass <= 0, the handler clamps it to 1.0. This guards the divide-by-
// mass dynamics. (inert = 50, upper = 80 => -30 -> clamped to 1.0)
// This exercises the clamp branch reachable through SeparationEvent::Apply.
// ---------------------------------------------------------------------------
TEST(FlightEvent, SeparationClampsNonPositiveInertMass) {
    SeparationEvent sep(90.0, 80.0, SeparationEvent::Callback());

    Rocket rocket;
    rocket.mass.inert = 50.0;  // less than the separated mass
    rocket.mass.propellant = 0.0;
    DynamicsBase::state x = ZeroState();

    sep.Apply(rocket, x);

    EXPECT_DOUBLE_EQ(rocket.mass.inert, 1.0) << "non-positive inert clamps to 1.0";
}


// ---------------------------------------------------------------------------
// ParachuteOpenEvent::Apply -> Rocket::OpenParachute(). With a single
// configured CdS stage, the first open adds that CdS; a second open (no more
// stages) leaves CdS unchanged. We configure CdS via setCdSParachute().
//   stage CdS = 3.0  => after first open CdS_parachute = 3.0,
//                       after second open still 3.0 (count exceeds src size).
// ---------------------------------------------------------------------------
TEST(FlightEvent, ParachuteOpenAddsConfiguredCdS) {
    ParachuteOpenEvent po(150.0);

    Rocket rocket;
    rocket.setCdSParachute(3.0);  // single drogue/main stage, CdS = 3.0
    rocket.CdS_parachute = 0.0;
    DynamicsBase::state x = ZeroState();

    po.Apply(rocket, x);
    EXPECT_DOUBLE_EQ(rocket.CdS_parachute, 3.0) << "first open adds stage CdS";

    // Second open: no further stage configured -> value unchanged.
    po.Apply(rocket, x);
    EXPECT_DOUBLE_EQ(rocket.CdS_parachute, 3.0) << "no extra stage -> unchanged";
}
