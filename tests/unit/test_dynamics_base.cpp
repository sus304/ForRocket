// ******************************************************
// Unit tests for src/dynamics/dynamics_base.cpp
//   Force / moment helper methods that take a Rocket*:
//     AeroForce, GyroEffectMoment, ThrustMoment, AeroForceMoment,
//     AeroDampingMoment, JetDampingMoment, GasJetMoment(rocket,t),
//     QuaternionDiff.
//
// DynamicsBase is abstract (pure-virtual operator()), so the tests use a
// trivial concrete subclass (TestDynamics) whose operator() is a no-op; that
// lets us exercise the protected/public helper methods directly.
//
// Strategy (aerospace correctness bar): wherever possible the expected value
// is an independent reference derived by hand from the physics, or an
// invariant (zero angular velocity -> gyro/damping moments vanish, zero
// thrust offset -> thrust moment vanishes, QuaternionDiff is skew-symmetric
// and norm-preserving, aero force scales with q*area*coeff). Every hardcoded
// number carries its derivation in a comment.
//
// Uses the shared Rocket builder in test_fixtures.hpp (read-only).
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>

#include "Eigen/Dense"

#include "dynamics/dynamics_base.hpp"

#include "test_fixtures.hpp"

namespace forrocket {

// Minimal concrete subclass so we can instantiate and reach the helpers.
class TestDynamics : public DynamicsBase {
  public:
    void operator()(const state& /*x*/, state& /*dx*/, const double /*t*/) override {}
};

using forrocket::test::MakeTestRocket;

// =====================================================================
// AeroForce
//   force_axial  = q * CA * area
//   force_normal = q * CNa * area
//   force_aero = (-force_axial,
//                 force_normal * sideslip_angle,
//                 -force_normal * angle_of_attack)
//   No internal branches; verify the linear scaling and signs.
// =====================================================================

TEST(DynamicsBase, AeroForce_ScalesWithDynamicPressureAreaCoeff) {
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    // Fixture: area = pi*0.15^2/4 = 0.0176714586764426 m^2, CA=0.3, CNa=10.0.
    r.CA = 0.3;
    r.CNa = 10.0;
    r.dynamic_pressure = 1000.0;     // [Pa]
    r.angle_of_attack = 0.05;        // [rad]
    r.sideslip_angle = 0.02;         // [rad]

    const double area = pi * 0.15 * 0.15 / 4.0;  // = 0.01767145867644258
    Eigen::Vector3d f = dyn.AeroForce(&r);

    // Axial: -q*CA*area = -1000*0.3*0.01767145867644258 = -5.301437602932774 N
    const double force_axial = 1000.0 * 0.3 * area;
    EXPECT_NEAR(f(0), -force_axial, 1e-9);

    // Normal magnitude: q*CNa*area = 1000*10*0.01767... = 176.7145867644258 N
    const double force_normal = 1000.0 * 10.0 * area;
    // y-component = force_normal * sideslip_angle = 176.714...*0.02 = 3.534291735288517
    EXPECT_NEAR(f(1), force_normal * 0.02, 1e-9);
    // z-component = -force_normal * angle_of_attack = -176.714...*0.05 = -8.83572933822129
    EXPECT_NEAR(f(2), -force_normal * 0.05, 1e-9);
}

TEST(DynamicsBase, AeroForce_ZeroDynamicPressureGivesZeroForce) {
    // Invariant: no dynamic pressure -> no aerodynamic force regardless of AoA.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.dynamic_pressure = 0.0;
    r.angle_of_attack = 0.3;
    r.sideslip_angle = 0.1;
    Eigen::Vector3d f = dyn.AeroForce(&r);
    EXPECT_NEAR(f.norm(), 0.0, 1e-12);
}

TEST(DynamicsBase, AeroForce_ScalesLinearlyWithDynamicPressure) {
    // Invariant: doubling q doubles the whole force vector (pure linear scaling).
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.CA = 0.3; r.CNa = 10.0;
    r.angle_of_attack = 0.05; r.sideslip_angle = 0.02;
    r.dynamic_pressure = 500.0;
    Eigen::Vector3d f1 = dyn.AeroForce(&r);
    r.dynamic_pressure = 1000.0;
    Eigen::Vector3d f2 = dyn.AeroForce(&r);
    EXPECT_TRUE(f2.isApprox(2.0 * f1, 1e-12));
}

// =====================================================================
// GyroEffectMoment  =  -(omega x (I*omega))
//   Reads getInertiaTensor() (burning branch) and angular_velocity.
//   Decisions exercised: burning vs not-burning inertia source.
// =====================================================================

TEST(DynamicsBase, GyroEffect_ZeroAngularVelocityIsZero) {
    // Invariant: with omega = 0 the gyroscopic moment is identically zero.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.angular_velocity << 0.0, 0.0, 0.0;
    Eigen::Vector3d m = dyn.GyroEffectMoment(&r);
    EXPECT_NEAR(m.norm(), 0.0, 1e-12);
}

TEST(DynamicsBase, GyroEffect_SpinAboutPrincipalAxisIsZero) {
    // Invariant: spin purely about a principal (here roll/x) axis of a diagonal
    // inertia tensor gives angular_momentum parallel to omega, so the cross
    // product (and the moment) vanish.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.engine.Cutoff();  // not burning -> uses stored diagonal inertia_tensor
    r.angular_velocity << 7.0, 0.0, 0.0;
    Eigen::Vector3d m = dyn.GyroEffectMoment(&r);
    EXPECT_NEAR(m.norm(), 0.0, 1e-12);
}

TEST(DynamicsBase, GyroEffect_DiagonalInertiaHandComputed) {
    // Independent reference for the Euler gyroscopic term with a DIAGONAL
    // inertia tensor I = diag(Ixx,Iyy,Izz):
    //   H = I*omega = (Ixx*p, Iyy*q, Izz*r)
    //   omega x H = ( q*Izz*r - r*Iyy*q,
    //                 r*Ixx*p - p*Izz*r,
    //                 p*Iyy*q - q*Ixx*p )
    //   moment = -(omega x H)
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.engine.Cutoff();  // stored diagonal inertia (Ixx=0.1, Iyy=Izz=5.0)
    const double Ixx = 0.1, Iyy = 5.0, Izz = 5.0;
    const double p = 1.0, q = 2.0, s = 3.0;  // s == r (avoid shadow of Rocket r)
    r.angular_velocity << p, q, s;

    // H = (0.1*1, 5*2, 5*3) = (0.1, 10, 15)
    // omega x H = (q*Hz - r*Hy, r*Hx - p*Hz, p*Hy - q*Hx)
    //           = (2*15 - 3*10, 3*0.1 - 1*15, 1*10 - 2*0.1)
    //           = (0.0, -14.7, 9.8)
    // moment = -(...) = (0.0, 14.7, -9.8)
    Eigen::Vector3d m = dyn.GyroEffectMoment(&r);
    Eigen::Vector3d H(Ixx * p, Iyy * q, Izz * s);
    Eigen::Vector3d expected = -1.0 * Eigen::Vector3d(p, q, s).cross(H);
    EXPECT_TRUE(m.isApprox(expected, 1e-12));
    EXPECT_NEAR(m(0), 0.0, 1e-12);
    EXPECT_NEAR(m(1), 14.7, 1e-12);
    EXPECT_NEAR(m(2), -9.8, 1e-12);
}

TEST(DynamicsBase, GyroEffect_BurningUsesEngineInertiaSource) {
    // Decision: getInertiaTensor() returns the burning-branch (src-built)
    // tensor while the engine is burning. The fixture sets the src tensor to
    // the SAME diagonal as the stored tensor, so the gyro moment is identical
    // whether burning or not -> confirms both code paths produce the expected
    // diagonal result.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.angular_velocity << 1.0, 2.0, 3.0;

    r.engine.Ignittion();                            // burning branch
    Eigen::Vector3d m_burn = dyn.GyroEffectMoment(&r);
    r.engine.Cutoff();                               // stored branch
    Eigen::Vector3d m_stored = dyn.GyroEffectMoment(&r);
    EXPECT_TRUE(m_burn.isApprox(m_stored, 1e-12));
    EXPECT_NEAR(m_burn(1), 14.7, 1e-12);             // same hand value as above
}

// =====================================================================
// ThrustMoment = thrust x moment_arm
//   moment_arm = (length_CG - length_thrust,
//                 y_CG - y_thrust_offset,
//                 z_CG - z_thrust_offset)
//   No branch; verify offset-zero invariant and a hand-computed offset case.
// =====================================================================

TEST(DynamicsBase, ThrustMoment_NoLateralOffsetAndAxialArmGivesZero) {
    // Invariant: an axial thrust acting along x with a purely axial moment arm
    // (no lateral CG/thrust offset) produces ZERO moment, because thrust x arm
    // of two parallel x-vectors is zero.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.force.thrust << 10000.0, 0.0, 0.0;  // pure axial thrust
    r.length_CG = 1.0; r.length_thrust = 0.0;  // arm_x = 1.0
    r.y_CG = 0.0; r.y_thrust_offset = 0.0;     // arm_y = 0
    r.z_CG = 0.0; r.z_thrust_offset = 0.0;     // arm_z = 0
    Eigen::Vector3d m = dyn.ThrustMoment(&r);
    EXPECT_NEAR(m.norm(), 0.0, 1e-9);
}

TEST(DynamicsBase, ThrustMoment_LateralThrustOffsetHandComputed) {
    // Independent reference. moment = thrust x arm.
    // arm = (length_CG - length_thrust, y_CG - y_thrust_offset,
    //        z_CG - z_thrust_offset)
    //     = (1.0 - 0.0, 0.0 - 0.1, 0.0 - (-0.2)) = (1.0, -0.1, 0.2)
    // thrust = (10000, 0, 0)
    // thrust x arm = ( 0*0.2 - 0*(-0.1),         = 0
    //                  0*1.0 - 10000*0.2,        = -2000
    //                  10000*(-0.1) - 0*1.0 )    = -1000
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.force.thrust << 10000.0, 0.0, 0.0;
    r.length_CG = 1.0; r.length_thrust = 0.0;
    r.y_CG = 0.0; r.y_thrust_offset = 0.1;
    r.z_CG = 0.0; r.z_thrust_offset = -0.2;

    Eigen::Vector3d m = dyn.ThrustMoment(&r);
    Eigen::Vector3d arm(1.0, -0.1, 0.2);
    Eigen::Vector3d thrust(10000.0, 0.0, 0.0);
    Eigen::Vector3d expected = thrust.cross(arm);
    EXPECT_TRUE(m.isApprox(expected, 1e-9));
    EXPECT_NEAR(m(0), 0.0, 1e-9);
    EXPECT_NEAR(m(1), -2000.0, 1e-9);
    EXPECT_NEAR(m(2), -1000.0, 1e-9);
}

// =====================================================================
// AeroForceMoment
//   moment = force.aero x (length_CG - length_CP, 0, 0)
//   then x-component is OVERWRITTEN with the roll moment:
//     moment[0] = q * Cld * area * diameter * cant_angle_fin
//   No branch; verify the overwrite and the pitch/yaw cross product.
// =====================================================================

TEST(DynamicsBase, AeroForceMoment_PitchYawFromCrossAndRollOverwrite) {
    // Independent reference.
    // arm = (length_CG - length_CP, 0, 0) = (1.0 - 1.2, 0, 0) = (-0.2, 0, 0)
    // force.aero set to (fx, fy, fz) = (-5, 3, -8)
    // aero x arm = ( fy*0 - fz*0,                 = 0   (then overwritten)
    //                fz*armx - fx*0,              = fz*armx = -8*(-0.2) = 1.6
    //                fx*0 - fy*armx )             = -fy*armx = -3*(-0.2) = 0.6
    // Then moment[0] = q*Cld*area*diameter*cant_angle_fin.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.length_CG = 1.0; r.length_CP = 1.2;  // arm_x = -0.2
    r.force.aero << -5.0, 3.0, -8.0;
    r.dynamic_pressure = 1000.0;
    r.Cld = 0.4;
    r.cant_angle_fin = 0.01;  // [rad]
    // area = pi*0.15^2/4 = 0.01767145867644258, diameter = 0.15
    const double area = pi * 0.15 * 0.15 / 4.0;
    const double diameter = 0.15;

    Eigen::Vector3d m = dyn.AeroForceMoment(&r);

    // roll: 1000*0.4*0.01767145867644258*0.15*0.01 = 0.0010602875205865549
    const double roll = 1000.0 * 0.4 * area * diameter * 0.01;
    EXPECT_NEAR(m(0), roll, 1e-12);
    // pitch (y): fz*arm_x = -8 * (-0.2) = 1.6
    EXPECT_NEAR(m(1), 1.6, 1e-12);
    // yaw (z): -fy*arm_x = -3 * (-0.2) = 0.6
    EXPECT_NEAR(m(2), 0.6, 1e-12);
}

TEST(DynamicsBase, AeroForceMoment_ZeroCantAngleGivesZeroRoll) {
    // Invariant: with no fin cant the roll-moment overwrite is zero, so the
    // x-component is zero even though the y/z cross-product terms are nonzero.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.length_CG = 1.0; r.length_CP = 1.2;
    r.force.aero << -5.0, 3.0, -8.0;
    r.dynamic_pressure = 1000.0;
    r.Cld = 0.4;
    r.cant_angle_fin = 0.0;  // no cant -> roll term zero
    Eigen::Vector3d m = dyn.AeroForceMoment(&r);
    EXPECT_NEAR(m(0), 0.0, 1e-12);
    EXPECT_NEAR(m(1), 1.6, 1e-12);  // y/z unaffected
    EXPECT_NEAR(m(2), 0.6, 1e-12);
}

// =====================================================================
// AeroDampingMoment
//   Decision 1: airspeed (= velocity.air_body.norm()) <= 0  -> returns ZERO.
//   Decision 2: airspeed > 0 -> componentwise
//     m_i = q * coeff_i * area * diameter^2 / (2*airspeed) * omega_i
//     coeff = (Clp, Cmq, Cnr)
// =====================================================================

TEST(DynamicsBase, AeroDamping_ZeroAirspeedReturnsZero) {
    // Decision 1 (airspeed <= 0): air_body left at zero -> zero damping moment,
    // even with nonzero angular velocity and dynamic pressure.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.velocity.air_body << 0.0, 0.0, 0.0;  // airspeed = 0
    r.angular_velocity << 1.0, 2.0, 3.0;
    r.dynamic_pressure = 1000.0;
    Eigen::Vector3d m = dyn.AeroDampingMoment(&r);
    EXPECT_NEAR(m.norm(), 0.0, 1e-12);
}

TEST(DynamicsBase, AeroDamping_ZeroAngularVelocityReturnsZero) {
    // Decision 2 reached (airspeed > 0) but omega = 0 -> each component is
    // multiplied by omega_i = 0, so the moment is zero.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.velocity.air_body << 100.0, 0.0, 0.0;  // airspeed = 100 > 0
    r.angular_velocity << 0.0, 0.0, 0.0;
    r.dynamic_pressure = 1000.0;
    Eigen::Vector3d m = dyn.AeroDampingMoment(&r);
    EXPECT_NEAR(m.norm(), 0.0, 1e-12);
}

TEST(DynamicsBase, AeroDamping_HandComputedComponents) {
    // Decision 2 with full nonzero inputs. Independent reference:
    //   m_i = q * coeff_i * area * diameter^2 / (2*airspeed) * omega_i
    // Fixture: Clp=-0.1, Cmq=-2.0, Cnr=-2.0 (the .cpp reads the raw members,
    // which we set directly here), area = pi*0.15^2/4, diameter = 0.15.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.Clp = -0.1; r.Cmq = -2.0; r.Cnr = -2.0;
    r.dynamic_pressure = 1000.0;
    r.velocity.air_body << 100.0, 0.0, 0.0;  // airspeed = 100
    r.angular_velocity << 1.0, 2.0, 3.0;     // p, q, r

    const double area = pi * 0.15 * 0.15 / 4.0;  // 0.01767145867644258
    const double d2 = 0.15 * 0.15;               // 0.0225
    const double airspeed = 100.0;
    const double base = 1000.0 * area * d2 / (2.0 * airspeed);
    // base = 1000 * 0.01767145867644258 * 0.0225 / 200 = 0.0019880391010997904
    Eigen::Vector3d expected;
    expected << base * (-0.1) * 1.0,   // roll:  -1.9880391e-4
                base * (-2.0) * 2.0,   // pitch: -7.9521564e-3
                base * (-2.0) * 3.0;   // yaw:   -1.19282346e-2

    Eigen::Vector3d m = dyn.AeroDampingMoment(&r);
    EXPECT_TRUE(m.isApprox(expected, 1e-12))
        << "got: " << m.transpose() << "  expected: " << expected.transpose();
}

TEST(DynamicsBase, AeroDamping_ScalesInverselyWithAirspeed) {
    // Invariant: the prefactor is proportional to 1/(2*airspeed); doubling
    // airspeed halves the damping moment (all other inputs fixed).
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.Clp = -0.1; r.Cmq = -2.0; r.Cnr = -2.0;
    r.dynamic_pressure = 1000.0;
    r.angular_velocity << 1.0, 2.0, 3.0;

    r.velocity.air_body << 100.0, 0.0, 0.0;
    Eigen::Vector3d m1 = dyn.AeroDampingMoment(&r);
    r.velocity.air_body << 200.0, 0.0, 0.0;
    Eigen::Vector3d m2 = dyn.AeroDampingMoment(&r);
    EXPECT_TRUE(m2.isApprox(0.5 * m1, 1e-12));
}

// =====================================================================
// JetDampingMoment  -- always zero (no inputs, no branch).
// =====================================================================

TEST(DynamicsBase, JetDamping_IsAlwaysZero) {
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.angular_velocity << 1.0, 2.0, 3.0;  // should not matter
    Eigen::Vector3d m = dyn.JetDampingMoment(&r);
    EXPECT_NEAR(m.norm(), 0.0, 1e-15);
}

// =====================================================================
// GasJetMoment(rocket, t)
//   Decision 1: !gas_jet_config.enable          -> zero
//   Decision 2: elapsed = t - time_launch_clear;
//               (elapsed >= 0.0 && elapsed <= duration) -> moment[0]=rolling_moment
//   Branches to cover: disabled; enabled & before launch clear (elapsed<0);
//   enabled & within window; enabled & after window (elapsed>duration);
//   plus the two inclusive window boundaries (elapsed==0, elapsed==duration).
// =====================================================================

TEST(DynamicsBase, GasJet_DisabledReturnsZero) {
    // Decision 1: enable == false short-circuits to zero regardless of timing.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.gas_jet_config.enable = false;
    r.gas_jet_config.rolling_moment = 5.0;
    r.gas_jet_config.duration = 2.5;
    r.time_launch_clear = 1.0;
    Eigen::Vector3d m = dyn.GasJetMoment(&r, 2.0);  // would be in-window if enabled
    EXPECT_NEAR(m.norm(), 0.0, 1e-15);
}

TEST(DynamicsBase, GasJet_EnabledBeforeLaunchClearIsZero) {
    // Decision 2 false via elapsed < 0 (t before launch clear).
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.gas_jet_config.enable = true;
    r.gas_jet_config.rolling_moment = 5.0;
    r.gas_jet_config.duration = 2.5;
    r.time_launch_clear = 3.0;
    Eigen::Vector3d m = dyn.GasJetMoment(&r, 1.0);  // elapsed = -2.0 < 0
    EXPECT_NEAR(m.norm(), 0.0, 1e-15);
}

TEST(DynamicsBase, GasJet_EnabledWithinWindowAppliesRollingMoment) {
    // Decision 2 true: 0 <= elapsed <= duration -> roll moment on +x only.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.gas_jet_config.enable = true;
    r.gas_jet_config.rolling_moment = 5.0;
    r.gas_jet_config.duration = 2.5;
    r.time_launch_clear = 1.0;
    Eigen::Vector3d m = dyn.GasJetMoment(&r, 2.0);  // elapsed = 1.0 in [0, 2.5]
    EXPECT_NEAR(m(0), 5.0, 1e-12);
    EXPECT_NEAR(m(1), 0.0, 1e-15);
    EXPECT_NEAR(m(2), 0.0, 1e-15);
}

TEST(DynamicsBase, GasJet_EnabledAfterWindowIsZero) {
    // Decision 2 false via elapsed > duration.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.gas_jet_config.enable = true;
    r.gas_jet_config.rolling_moment = 5.0;
    r.gas_jet_config.duration = 2.5;
    r.time_launch_clear = 1.0;
    Eigen::Vector3d m = dyn.GasJetMoment(&r, 5.0);  // elapsed = 4.0 > 2.5
    EXPECT_NEAR(m.norm(), 0.0, 1e-15);
}

TEST(DynamicsBase, GasJet_WindowBoundariesAreInclusive) {
    // The >= 0.0 and <= duration comparisons are inclusive: both boundaries
    // (elapsed == 0 and elapsed == duration) apply the rolling moment.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.gas_jet_config.enable = true;
    r.gas_jet_config.rolling_moment = 5.0;
    r.gas_jet_config.duration = 2.5;
    r.time_launch_clear = 1.0;

    Eigen::Vector3d m_start = dyn.GasJetMoment(&r, 1.0);  // elapsed = 0.0
    EXPECT_NEAR(m_start(0), 5.0, 1e-12);

    Eigen::Vector3d m_end = dyn.GasJetMoment(&r, 3.5);    // elapsed = 2.5 == duration
    EXPECT_NEAR(m_end(0), 5.0, 1e-12);
}

// =====================================================================
// QuaternionDiff  -- the 4x4 quaternion kinematics matrix Omega(omega):
//   [  0   r  -q   p ]
//   [ -r   0   p   q ]
//   [  q  -p   0   r ]
//   [ -p  -q  -r   0 ]
//   No branch. Invariants: skew-symmetric (=> q^T (Omega q_dot) preserves
//   norm), and exact entries from omega.
// =====================================================================

TEST(DynamicsBase, QuaternionDiff_IsSkewSymmetric) {
    // Skew-symmetry (M^T = -M) guarantees the quaternion-derivative map
    // q_dot = 0.5 * M * q preserves ||q|| (d/dt ||q||^2 = q^T M q = 0).
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.angular_velocity << 0.3, -0.7, 1.1;
    Eigen::Matrix4d M = dyn.QuaternionDiff(&r);
    EXPECT_TRUE(M.transpose().isApprox(-M, 1e-15)) << M;
    // Diagonal of a skew matrix is zero.
    for (int i = 0; i < 4; ++i) EXPECT_NEAR(M(i, i), 0.0, 1e-15);
}

TEST(DynamicsBase, QuaternionDiff_NormPreservingOnUnitQuaternion) {
    // Invariant: for q_dot = 0.5 * M * q, the rate of change of ||q||^2 is
    // q . q_dot = 0 (because M is skew-symmetric). Check q . (M q) == 0.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.angular_velocity << 0.3, -0.7, 1.1;
    Eigen::Matrix4d M = dyn.QuaternionDiff(&r);
    Eigen::Vector4d q(0.1, 0.2, 0.3, 0.4);
    q.normalize();
    Eigen::Vector4d q_dot = 0.5 * M * q;
    EXPECT_NEAR(q.dot(q_dot), 0.0, 1e-15);
}

TEST(DynamicsBase, QuaternionDiff_MatrixEntriesFromAngularVelocity) {
    // Independent reference of every nonzero entry.
    // p=0.3, q=-0.7, r=1.1
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    const double p = 0.3, qq = -0.7, rr = 1.1;
    r.angular_velocity << p, qq, rr;
    Eigen::Matrix4d M = dyn.QuaternionDiff(&r);

    Eigen::Matrix4d expected;
    expected <<  0.0,  rr,  -qq,  p,
                -rr,  0.0,   p,  qq,
                 qq,  -p,  0.0,  rr,
                 -p, -qq,  -rr, 0.0;
    EXPECT_TRUE(M.isApprox(expected, 1e-15)) << M;
}

TEST(DynamicsBase, QuaternionDiff_ZeroAngularVelocityIsZeroMatrix) {
    // Invariant: omega = 0 -> the kinematics matrix is identically zero, so an
    // attitude with no body rates has zero quaternion derivative.
    TestDynamics dyn;
    Rocket r = MakeTestRocket();
    r.angular_velocity << 0.0, 0.0, 0.0;
    Eigen::Matrix4d M = dyn.QuaternionDiff(&r);
    EXPECT_TRUE(M.isApprox(Eigen::Matrix4d::Zero(), 1e-15)) << M;
}

}  // namespace forrocket
