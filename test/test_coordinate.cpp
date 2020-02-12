#include "gtest/gtest.h"

#include <iostream>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

#include "environment/coordinate.hpp"
#include "environment/sequence_clock.hpp"
#include "factory/rocket_factory.hpp"


TEST(CoordinateTest, PositionInitilize) {
    namespace fr = forrocket;

    fr::SequenceClock clock;
    fr::RocketFactory rf;

    auto rocket = rf.Create("f", "e");
    
    Eigen::Vector3d pllh;
    pllh << 35.2, 135.3, 0.0;
    rocket.position.Initialize(clock.UTC_date_init, pllh);

    EXPECT_EQ(pllh, rocket.position.LLH);
}

TEST(CoordinateTest, PositionLLHECEFTranslate) {
    namespace fr = forrocket;
    fr::Coordinate coord;

    Eigen::Vector3d llh;
    llh << 35.2, 135.3, 0.0;

    auto ecef = coord.LLH2ECEF(llh);
    auto llh_aft = coord.ECEF2LLH(ecef);

    std::cout << llh << std::endl;
    std::cout << ecef << std::endl;
    std::cout << llh_aft << std::endl;

    EXPECT_EQ(llh, llh_aft);

}

TEST(CoordinateTest, PositionTranslate) {
    namespace fr = forrocket;
    fr::Coordinate coord;

    Eigen::Vector3d llh;
    llh << 35.2, 135.3, 0.0;

    auto ecef = coord.LLH2ECEF(llh);
    coord.setECI2ECEF(10.0);
    auto eci = coord.dcm.ECEF2ECI * ecef;

    auto ecef_aft = coord.dcm.ECI2ECEF * eci;
    auto llh_aft = coord.ECEF2LLH(ecef_aft);

    std::cout << llh << std::endl;
    std::cout << llh_aft << std::endl;

    EXPECT_EQ(llh, llh_aft);

}