// ******************************************************
// Unit tests for src/rocket/engine.cpp
//   - 4 constructors (fixed / time-vector thrust, with / without misalignment)
//   - Update (ignition vs cutoff conditions, pressure-corrected thrust)
//   - Ignittion / Cutoff
// ******************************************************

#include <gtest/gtest.h>
#include <vector>

#include "rocket/engine.hpp"

using forrocket::Engine;

TEST(Engine, ConstThrustConstructor) {
    Engine e(10.0, 10000.0, 4.0, 0.01);
    EXPECT_DOUBLE_EQ(e.burn_duration, 10.0);
    EXPECT_DOUBLE_EQ(e.total_impulse, 100000.0);  // thrust * burn_duration
    EXPECT_FALSE(e.burning);
}

TEST(Engine, ConstThrustWithMisalignment) {
    Engine e(10.0, 10000.0, 4.0, 0.01, 0.02, -0.03);
    e.Update(1.0, 0.0, 5.0);  // burning
    EXPECT_DOUBLE_EQ(e.gimbal_angle_y_axis, 0.02);
    EXPECT_DOUBLE_EQ(e.gimbal_angle_z_axis, -0.03);
}

TEST(Engine, TimeVectorConstructor) {
    std::vector<double> t   = {0.0, 1.0, 2.0};
    std::vector<double> thr = {1000.0, 2000.0, 3000.0};
    std::vector<double> md  = {1.0, 1.0, 1.0};
    Engine e(t, thr, md, 0.0);
    EXPECT_DOUBLE_EQ(e.burn_duration, 2.0);            // time.back()
    EXPECT_DOUBLE_EQ(e.total_impulse, 2000.0 * 2.0);   // mean(thrust) * burn_duration
}

TEST(Engine, TimeVectorWithMisalignment) {
    std::vector<double> t   = {0.0, 2.0};
    std::vector<double> thr = {1000.0, 1000.0};
    std::vector<double> md  = {2.0, 2.0};
    Engine e(t, thr, md, 0.0, 0.01, 0.02);
    EXPECT_DOUBLE_EQ(e.burn_duration, 2.0);
    e.Update(1.0, 0.0, 5.0);
    EXPECT_TRUE(e.burning);
    EXPECT_DOUBLE_EQ(e.gimbal_angle_y_axis, 0.01);
    EXPECT_DOUBLE_EQ(e.gimbal_angle_z_axis, 0.02);
}

TEST(Engine, UpdateBurningWithPressureCorrection) {
    Engine e(10.0, 10000.0, 4.0, 0.01);
    e.Update(1.0, 1000.0, 5.0);  // t<=dur && mass_prop>0 -> burning
    EXPECT_TRUE(e.burning);
    EXPECT_DOUBLE_EQ(e.thrust, 10000.0 - 1000.0 * 0.01);  // vacuum - pressure*area_exit
    EXPECT_DOUBLE_EQ(e.mdot_prop, 4.0);
}

TEST(Engine, UpdateCutoffAfterBurnDuration) {
    Engine e(10.0, 10000.0, 4.0, 0.01);
    e.Update(11.0, 0.0, 5.0);  // t > burn_duration -> cutoff
    EXPECT_FALSE(e.burning);
    EXPECT_DOUBLE_EQ(e.thrust, 0.0);
    EXPECT_DOUBLE_EQ(e.mdot_prop, 0.0);
}

TEST(Engine, UpdateCutoffWhenPropellantExhausted) {
    Engine e(10.0, 10000.0, 4.0, 0.01);
    e.Update(1.0, 0.0, 0.0);  // mass_prop<=0 -> cutoff even within burn time
    EXPECT_FALSE(e.burning);
    EXPECT_DOUBLE_EQ(e.thrust, 0.0);
}

TEST(Engine, IgnittionAndCutoff) {
    Engine e(10.0, 10000.0, 4.0, 0.01);
    EXPECT_FALSE(e.burning);
    e.Ignittion();
    EXPECT_TRUE(e.burning);
    e.Cutoff();
    EXPECT_FALSE(e.burning);
}
