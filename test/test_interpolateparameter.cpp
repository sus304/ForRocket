#include "gtest/gtest.h"

#include <iostream>
#include <string>
#include <vector>

#include "../src/rocket/parameter/interpolate_parameter.hpp"

TEST(InterpolateParamterTest, Nominal1DLog) {
    namespace fr = forrocket;
    
    std::vector<double> x{0.0, 30.6};
    std::vector<double> y{0.1, 5.1};

    auto ip = fr::InterpolateParameter(x, y, "same");
    EXPECT_EQ(ip(15.3), 2.6);
    EXPECT_EQ(ip(-1.0), 0.1);
    EXPECT_EQ(ip(30.9), 5.1);

}

