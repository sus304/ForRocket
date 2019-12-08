// ******************************************************
// Project Name    : ForRocket
// File Name       : attitude.cpp
// Creation Date   : 2019/12/04
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "attitude.hpp"

#include "environment/coordinate.hpp"

forrocket::Attitude::Attitude() {
    euler_angle << 0.0, 0.0, 0.0;
    quaternion << 0.0, 0.0, 0.0, 0.0;
};


void forrocket::Attitude::Initialize(const Eigen::Vector3d& euler_angle) {
    Coordinate coordinate;
    this->euler_angle = euler_angle;
    this->quaternion = coordinate.Quaternion(this->euler_angle);
};


void forrocket::Attitude::Update(const Eigen::Vector4d& quaternion, Coordinate& coordinate) {
    this->quaternion = quaternion.normalized();
    this->euler_angle = coordinate.EulerAngle();
};


