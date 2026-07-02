// ******************************************************
// Project Name    : ForRocket
// File Name       : gravity.hpp
// Creation Date   : 2019/10/27
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ENVIRONMENT_GRAVITY_HPP_
#define ENVIRONMENT_GRAVITY_HPP_

#include <cmath>

#include "Eigen/Core"

#include "environment/wgs84.hpp"

namespace forrocket {
    inline double gravity(const double altitude) {
        WGS84 wgs84;
        double geocentric_height;
        if (altitude < 0.0) geocentric_height = wgs84.a;
        else geocentric_height = altitude + wgs84.a;
        return wgs84.GM / std::pow(geocentric_height, 2);
    };

    // 質点 + J2帯状調和項の重力加速度 [m/s2] を ECEF 系で返す（遠心力は含まない。
    // ECI で積分するため引力のみでよい）。"Gravity Model": "pointmass-j2" で使用。
    Eigen::Vector3d gravityECEF(const Eigen::Vector3d& position_ECEF);
}

#endif

