// ******************************************************
// Unit tests for src/rocket/rocket.cpp
//   - coefficient getters (burning/burnout branches, sign-forced damping)
//   - getThrust / getInertiaTensor (burning branches)
//   - SOE handlers (ignition, cutoff, despin, jettison, separation, parachute)
// Uses the shared Rocket builder in test_fixtures.hpp.
// ******************************************************

#include <gtest/gtest.h>

#include "Eigen/Dense"

#include "rocket/rocket.hpp"
#include "rocket/parameter/interpolate_parameter.hpp"
#include "environment/datetime.hpp"

#include "test_fixtures.hpp"

using forrocket::Rocket;
using forrocket::InterpolateParameter;
using forrocket::test::MakeTestRocket;

TEST(Rocket, GetCABurningVsBurnout) {
    Rocket r = MakeTestRocket();
    r.engine.Cutoff();
    EXPECT_DOUBLE_EQ(r.getCA(0.5), 0.3);  // burnout branch (CA_burnout_src)
    r.engine.Ignittion();
    EXPECT_DOUBLE_EQ(r.getCA(0.5), 0.3);  // burning branch (CA_src)
}

TEST(Rocket, GetLengthCGBurningBranch) {
    Rocket r = MakeTestRocket();
    r.engine.Cutoff();
    r.length_CG = 1.5;
    EXPECT_DOUBLE_EQ(r.getLengthCG(), 1.5);  // not burning -> stored value
    r.engine.Ignittion();
    EXPECT_DOUBLE_EQ(r.getLengthCG(), 1.0);  // burning -> reads src (=1.0)
}

TEST(Rocket, GetLateralCGMassWeighting) {
    Rocket r = MakeTestRocket();  // inert = 10, propellant = 5
    r.y_CG_inert = 1.5;
    r.z_CG_inert = -0.6;
    // Full propellant load: effective offset is diluted by the centerline propellant.
    //   y = 1.5 * 10 / (10 + 5) = 1.0 ;  z = -0.6 * 10 / 15 = -0.4
    EXPECT_DOUBLE_EQ(r.getYCG(), 1.0);
    EXPECT_DOUBLE_EQ(r.getZCG(), -0.4);
    EXPECT_DOUBLE_EQ(r.y_CG, 1.0);   // getter caches into the effective member
    EXPECT_DOUBLE_EQ(r.z_CG, -0.4);
    // Burned out: effective offset equals the inert (dry) structure offset.
    r.mass.propellant = 0.0;
    EXPECT_DOUBLE_EQ(r.getYCG(), 1.5);
    EXPECT_DOUBLE_EQ(r.getZCG(), -0.6);
}

TEST(Rocket, GetLateralCGZeroMassFallsBackToInert) {
    Rocket r = MakeTestRocket();
    r.y_CG_inert = 2.0;
    r.mass.inert = 0.0;
    r.mass.propellant = 0.0;  // degenerate total mass -> fall back to inert offset
    EXPECT_DOUBLE_EQ(r.getYCG(), 2.0);
}

TEST(Rocket, GetCNaAndLengthCP) {
    Rocket r = MakeTestRocket();
    EXPECT_DOUBLE_EQ(r.getCNa(0.5), 10.0);
    EXPECT_DOUBLE_EQ(r.getLengthCP(0.5), 1.2);
}

TEST(Rocket, DampingCoefficientsForcedNegative) {
    Rocket r = MakeTestRocket();
    // Clp/Cmq/Cnr are forced <= 0 (sign convention): positive src gets negated.
    r.setClp(InterpolateParameter(0.2));
    r.setCmq(InterpolateParameter(3.0));
    r.setCnr(InterpolateParameter(1.5));
    EXPECT_DOUBLE_EQ(r.getClp(0.5), -0.2);
    EXPECT_DOUBLE_EQ(r.getCmq(0.5), -3.0);
    EXPECT_DOUBLE_EQ(r.getCnr(0.5), -1.5);
    // Already-negative is left unchanged (else branch).
    r.setClp(InterpolateParameter(-0.4));
    EXPECT_DOUBLE_EQ(r.getClp(0.5), -0.4);
}

TEST(Rocket, GetThrustBurningVsCutoff) {
    Rocket r = MakeTestRocket();
    // burning: Update(countup=0, p=0, mass.prop=5) -> burning, axial thrust
    Eigen::Vector3d th = r.getThrust(0.0);
    EXPECT_GT(th(0), 0.0);
    EXPECT_NEAR(th(1), 0.0, 1e-9);
    EXPECT_NEAR(th(2), 0.0, 1e-9);
    // exhaust propellant -> cutoff -> zero thrust
    r.mass.propellant = 0.0;
    Eigen::Vector3d th2 = r.getThrust(0.0);
    EXPECT_DOUBLE_EQ(th2.norm(), 0.0);
}

TEST(Rocket, GetInertiaTensorBurningBranch) {
    Rocket r = MakeTestRocket();
    r.engine.Cutoff();
    Eigen::Matrix3d stored = r.getInertiaTensor();  // not burning -> stored
    EXPECT_DOUBLE_EQ(stored(0, 0), 0.1);
    r.engine.Ignittion();
    Eigen::Matrix3d t = r.getInertiaTensor();        // burning -> built from src
    EXPECT_DOUBLE_EQ(t(0, 0), 0.1);
    EXPECT_DOUBLE_EQ(t(1, 1), 5.0);
    EXPECT_DOUBLE_EQ(t(0, 1), 0.0);  // -Ixy, products are zero
}

TEST(Rocket, GetAttitudeProgramAndRate) {
    Rocket r = MakeTestRocket();
    r.setAttitudeProgram(InterpolateParameter(1.0), InterpolateParameter(2.0),
                         InterpolateParameter(3.0));
    Eigen::Vector3d a = r.getAttitude();
    EXPECT_DOUBLE_EQ(a(0), 1.0);
    EXPECT_DOUBLE_EQ(a(1), 2.0);
    EXPECT_DOUBLE_EQ(a(2), 3.0);

    r.setAttitudeProgramRate(InterpolateParameter(0.1), InterpolateParameter(0.2),
                             InterpolateParameter(0.3));
    Eigen::Vector3d rate = r.getAttitudeRate();
    EXPECT_DOUBLE_EQ(rate(0), 0.1);
    EXPECT_DOUBLE_EQ(rate(2), 0.3);
}

TEST(Rocket, IgnitionAndCutoffEngineSOE) {
    Rocket r = MakeTestRocket();
    r.engine.Cutoff();
    r.IgnitionEngine(forrocket::DateTime(), 0.0);
    EXPECT_TRUE(r.engine.burning);
    r.CutoffEngine();
    EXPECT_FALSE(r.engine.burning);
}

TEST(Rocket, DeSpinZerosRoll) {
    Rocket r = MakeTestRocket();
    r.angular_velocity[0] = 5.0;
    r.angular_acceleration[0] = 2.0;
    r.DeSpin();
    EXPECT_DOUBLE_EQ(r.angular_velocity[0], 0.0);
    EXPECT_DOUBLE_EQ(r.angular_acceleration[0], 0.0);
}

TEST(Rocket, JettisonAndSeparationClampInertMass) {
    Rocket r = MakeTestRocket();   // inert = 10
    r.JettsonFairing(3.0);
    EXPECT_DOUBLE_EQ(r.mass.inert, 7.0);
    r.JettsonFairing(100.0);       // would go <= 0 -> clamped to 1.0
    EXPECT_DOUBLE_EQ(r.mass.inert, 1.0);

    Rocket r2 = MakeTestRocket();
    r2.SeparateUpperStage(4.0);
    EXPECT_DOUBLE_EQ(r2.mass.inert, 6.0);
    r2.SeparateUpperStage(100.0);  // clamp
    EXPECT_DOUBLE_EQ(r2.mass.inert, 1.0);
}

TEST(Rocket, OpenParachuteAccumulatesUpToConfigured) {
    Rocket r = MakeTestRocket();
    r.setCdSParachute(2.0, 3.0);
    EXPECT_DOUBLE_EQ(r.CdS_parachute, 0.0);
    r.OpenParachute();
    EXPECT_DOUBLE_EQ(r.CdS_parachute, 2.0);  // first
    r.OpenParachute();
    EXPECT_DOUBLE_EQ(r.CdS_parachute, 5.0);  // + second
    r.OpenParachute();                        // count >= size -> no-op
    EXPECT_DOUBLE_EQ(r.CdS_parachute, 5.0);
}
