// ******************************************************
// Project Name    : ForRocket
// File Name       : force.cpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "force.hpp"

forrocket::Force::Force() {
    thrust << 0.0, 0.0, 0.0;
    aero << 0.0, 0.0, 0.0;
    gravity << 0.0, 0.0, 0.0;
};

Eigen::Vector3d forrocket::Force::Sum() {
    Eigen::Vector3d sum;
    sum = thrust + aero + gravity;
    return sum;
};

