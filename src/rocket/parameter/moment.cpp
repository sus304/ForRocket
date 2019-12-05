// ******************************************************
// Project Name    : ForRocket
// File Name       : moment.cpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "moment.hpp"

forrocket::Moment::Moment() {
    gyro << 0.0, 0.0, 0.0;
    thrust << 0.0, 0.0, 0.0;
    aero_force << 0.0, 0.0, 0.0;
    aero_dumping << 0.0, 0.0, 0.0;
    jet_dumping << 0.0, 0.0, 0.0;
};

Eigen::Vector3d forrocket::Moment::Sum() {
    Eigen::Vector3d sum;
    sum = gyro + thrust + aero_force + aero_dumping + jet_dumping;
    return sum;
};