#include "gtest/gtest.h"

#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

#include "environment/coordinate.hpp"
#include "environment/datetime.hpp"
#include "environment/sequence_clock.hpp"

#include "common4test.hpp"

TEST(ClockTest, DateTimeInitialize) {
    namespace fr = forrocket;

    fr::DateTime launch_date(2003, 10, 16, 9, 43, 0);
    EXPECT_EQ(launch_date.year, 2003);
    EXPECT_EQ(launch_date.month, 10);
    EXPECT_EQ(launch_date.day, 16);
    EXPECT_EQ(launch_date.hour, 9);
    EXPECT_EQ(launch_date.min, 43);
    EXPECT_EQ(launch_date.sec, 0);

    launch_date = fr::DateTime(1992, 8, 20, 12, 14, 0);
    EXPECT_EQ(launch_date.year, 1992);
    EXPECT_EQ(launch_date.month, 8);
    EXPECT_EQ(launch_date.day, 20);
    EXPECT_EQ(launch_date.hour, 12);
    EXPECT_EQ(launch_date.min, 14);
    EXPECT_EQ(launch_date.sec, 0);
}

TEST(ClockTest, EpocTime) {
    namespace fr = forrocket;
    fr::DateTime launch_date;
    fr::SequenceClock clock;
    
    launch_date = fr::DateTime(1992, 8, 20, 12, 14, 0);
    clock = fr::SequenceClock(launch_date, 0);
    EXPECT_EQ(order_floor(clock.greenwich_sidereal_time,4), order_floor(-232984181.090015915595,4));

    launch_date = fr::DateTime(2003, 10, 16, 9, 43, 0);
    clock = fr::SequenceClock(launch_date, 0);
    EXPECT_EQ(order_floor(clock.julian_data), order_floor(2452928.904861111));

    launch_date = fr::DateTime(2003, 10, 16, 9, 43, 22);
    clock = fr::SequenceClock(launch_date, 0);
    EXPECT_EQ(order_floor(clock.julian_data), order_floor(2452928.90511574));

}

