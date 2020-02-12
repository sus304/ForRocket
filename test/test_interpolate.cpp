#include "gtest/gtest.h"

#include <iostream>
#include <string>
#include <vector>

#include "../src/pylib.hpp"
#include "../src/interpolate.hpp"

TEST(InterpolateTest, Interp1dLinearNormalInput) {
    namespace fr = forrocket;
    
    std::vector<double> x;
    x.push_back(0.0);
    x.push_back(20.0);
    std::vector<double> y;
    y.push_back(1.0);
    y.push_back(2.0);

    auto polator = fr::interpolate::Interp1d(x, y, "linear", "extrapolate");

    EXPECT_EQ(polator(10.0), 1.5);
}

