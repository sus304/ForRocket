// ******************************************************
// Shared GoogleTest fixtures/builders for Tier B tests that need a constructed
// Rocket / Engine. Values are internally consistent and deterministic — NOT
// flight-realistic. Read-only: tests include this and build on top; they must
// not modify it (concurrent edits would clash).
// ******************************************************

#ifndef FORROCKET_TEST_FIXTURES_HPP_
#define FORROCKET_TEST_FIXTURES_HPP_

#include <vector>

#include "Eigen/Core"

#include "degrad.hpp"
#include "environment/datetime.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"
#include "rocket/engine.hpp"
#include "rocket/parameter/interpolate_parameter.hpp"
#include "rocket/rocket.hpp"

namespace forrocket {
namespace test {

// Constant-thrust engine: 10 kN vacuum thrust for 10 s, 4 kg/s mdot,
// 0.01 m^2 nozzle exit. total_impulse = 10000*10.
inline Engine MakeConstThrustEngine() {
    return Engine(/*burn_duration=*/10.0, /*thrust_const=*/10000.0,
                  /*mdot_prop_const=*/4.0, /*area_exit=*/0.01);
}

// Launch site used by the fixtures (matches the sample config ballpark).
inline Eigen::Vector3d LaunchLLH() {
    return Eigen::Vector3d(40.242865, 140.01045, 20.0);  // lat[deg], lon[deg], h[m]
}

// A reasonably-complete single-stage Rocket. Aero coefficients, mass and
// inertia are set as constants so the getters return deterministic values.
// Position/velocity/attitude are initialized at the launch site pointing up.
inline Rocket MakeTestRocket() {
    Rocket r;
    r.diameter = 0.15;
    r.area = pi * 0.15 * 0.15 / 4.0;
    r.length = 2.0;
    r.length_thrust = 0.0;

    r.engine = MakeConstThrustEngine();
    DateTime epoch;                                   // default epoch
    r.burn_clock = SequenceClock(epoch, 0.0);         // countup_time = 0

    r.mass.inert = 10.0;
    r.mass.propellant = 5.0;

    // Constant aerodynamic coefficients (burning + burnout share CA here).
    r.setCA(InterpolateParameter(0.3), InterpolateParameter(0.3));
    r.setCNa(InterpolateParameter(10.0));
    r.setCld(InterpolateParameter(0.0));
    r.setClp(InterpolateParameter(-0.1));
    r.setCmq(InterpolateParameter(-2.0));
    r.setCnr(InterpolateParameter(-2.0));
    r.setLengthCG(InterpolateParameter(1.0));
    r.setLengthCP(InterpolateParameter(1.2));

    // Inertia (diagonal): Ixx small (roll), Iyy=Izz larger (pitch/yaw).
    r.setInertiaTensor(InterpolateParameter(0.1), InterpolateParameter(5.0),
                       InterpolateParameter(5.0));
    r.setInertiaProduct(InterpolateParameter(0.0), InterpolateParameter(0.0),
                        InterpolateParameter(0.0));
    r.inertia_tensor << 0.1, 0.0, 0.0,
                        0.0, 5.0, 0.0,
                        0.0, 0.0, 5.0;

    // Kinematic state: at the launch site, elevation 85 deg, azimuth 270 deg.
    r.position.Initialize(epoch, LaunchLLH());
    r.velocity.Initialize(epoch, Eigen::Vector3d(0.0, 0.0, 0.0), LaunchLLH(),
                          r.position.ECI);
    r.attitude.Initialize(Eigen::Vector3d(deg2rad(270.0), deg2rad(85.0), deg2rad(0.0)));
    r.angular_velocity << 0.0, 0.0, 0.0;
    r.angular_acceleration << 0.0, 0.0, 0.0;

    return r;
}

// Zero-wind model (disabled), for dynamics tests that don't probe wind.
inline EnvironmentWind MakeZeroWind() {
    return EnvironmentWind(false);
}

}  // namespace test
}  // namespace forrocket

#endif
