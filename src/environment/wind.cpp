// ******************************************************
// Project Name    : ForRocket
// File Name       : wind.cpp
// Creation Date   : 2019/10/26
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "environment/wind.hpp"

Eigen::Vector3d forrocket::EnvironmentWind::getNED(const double altitude) {
    Eigen::Vector3d NED;

    NED << 0.0, 0.0, 0.0;

    return NED;
};
