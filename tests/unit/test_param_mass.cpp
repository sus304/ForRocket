// ******************************************************
// Unit tests for src/rocket/parameter/mass.cpp
//   - Mass::Sum() returns inert + propellant
// ******************************************************

#include <gtest/gtest.h>

#include "mass.hpp"

using forrocket::Mass;

// Decision points in mass.cpp:
//   Sum() has a single straight-line return (inert + propellant); no branches.
//   These tests exercise that one statement with several value combinations.

TEST(MassParam, SumAddsInertAndPropellant) {
    Mass m;
    m.inert = 10.0;
    m.propellant = 4.0;
    // Reference: inert + propellant = 10 + 4 = 14 (definition of Sum()).
    EXPECT_DOUBLE_EQ(m.Sum(), 14.0);
}

TEST(MassParam, SumWithZeroPropellant) {
    Mass m;
    m.inert = 7.5;
    m.propellant = 0.0;
    // Burnout case: total mass equals the inert (dry) mass.
    EXPECT_DOUBLE_EQ(m.Sum(), 7.5);
}

TEST(MassParam, SumWithZeroInert) {
    Mass m;
    m.inert = 0.0;
    m.propellant = 3.25;
    EXPECT_DOUBLE_EQ(m.Sum(), 3.25);
}

TEST(MassParam, SumBothZero) {
    Mass m;
    m.inert = 0.0;
    m.propellant = 0.0;
    EXPECT_DOUBLE_EQ(m.Sum(), 0.0);
}
