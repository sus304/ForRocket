// ******************************************************
// Unit tests for inc/environment/wgs84.hpp (WGS84 constants).
// ******************************************************

#include <gtest/gtest.h>
#include <cmath>

#include "environment/wgs84.hpp"

using forrocket::WGS84;

TEST(WGS84, DefiningConstants) {
    WGS84 wgs;
    EXPECT_DOUBLE_EQ(wgs.a, 6378137.0);
    EXPECT_DOUBLE_EQ(wgs.inv_f, 298.257223563);
    EXPECT_DOUBLE_EQ(wgs.GM, 3.986004418e14);
    EXPECT_DOUBLE_EQ(wgs.omega, 7292115e-11);
}

TEST(WGS84, DerivedConstantsAreConsistent) {
    WGS84 wgs;
    // f = 1/inv_f, b = a(1-f), e^2 = 2f - f^2, e = sqrt(e^2)
    EXPECT_DOUBLE_EQ(wgs.f, 1.0 / wgs.inv_f);
    EXPECT_DOUBLE_EQ(wgs.b, wgs.a * (1.0 - wgs.f));
    EXPECT_DOUBLE_EQ(wgs.e_square, 2.0 * wgs.f - wgs.f * wgs.f);
    EXPECT_DOUBLE_EQ(wgs.e, std::sqrt(wgs.e_square));
    // sqrt(1 - e^2) == (1 - f) is the geometric identity tying b to e.
    EXPECT_NEAR(std::sqrt(1.0 - wgs.e_square), 1.0 - wgs.f, 1e-15);
}

TEST(WGS84, DerivedConstantsMatchPublishedReference) {
    WGS84 wgs;
    EXPECT_NEAR(wgs.b, 6356752.314245, 1e-3);          // semi-minor axis [m]
    EXPECT_NEAR(wgs.e_square, 0.0066943799901413165, 1e-15);
    EXPECT_NEAR(wgs.e, 0.0818191908426215, 1e-12);
}
