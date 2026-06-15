// ******************************************************
// Unit tests for src/degrad.cpp (deg<->rad conversion).
// ******************************************************

#include <gtest/gtest.h>

#include "degrad.hpp"

using forrocket::deg2rad;
using forrocket::rad2deg;
using forrocket::pi;

TEST(Degrad, Deg2RadKnownValues) {
    EXPECT_NEAR(deg2rad(0.0), 0.0, 1e-15);
    EXPECT_NEAR(deg2rad(90.0), pi / 2.0, 1e-12);
    EXPECT_NEAR(deg2rad(180.0), pi, 1e-12);
    EXPECT_NEAR(deg2rad(360.0), 2.0 * pi, 1e-12);
    EXPECT_NEAR(deg2rad(-90.0), -pi / 2.0, 1e-12);
}

TEST(Degrad, Rad2DegKnownValues) {
    EXPECT_NEAR(rad2deg(0.0), 0.0, 1e-15);
    EXPECT_NEAR(rad2deg(pi / 2.0), 90.0, 1e-12);
    EXPECT_NEAR(rad2deg(pi), 180.0, 1e-12);
    EXPECT_NEAR(rad2deg(2.0 * pi), 360.0, 1e-12);
}

TEST(Degrad, RoundTrip) {
    for (double deg = -350.0; deg <= 350.0; deg += 17.0) {
        EXPECT_NEAR(rad2deg(deg2rad(deg)), deg, 1e-10);
    }
}
