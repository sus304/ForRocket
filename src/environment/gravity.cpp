// ******************************************************
// Project Name    : ForRocket
// File Name       : gravity.cpp
// Creation Date   : 2019/10/27
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "environment/gravity.hpp"

Eigen::Vector3d forrocket::gravityECEF(const Eigen::Vector3d& position_ECEF) {
    // 質点項 -GM/r^3 * r に J2 帯状調和項を加えた引力加速度（ECEF成分）。
    //   g_x = -GM*x/r^3 * (1 + 3/2*J2*(a/r)^2 * (1 - 5*(z/r)^2))
    //   g_y = -GM*y/r^3 * (1 + 3/2*J2*(a/r)^2 * (1 - 5*(z/r)^2))
    //   g_z = -GM*z/r^3 * (1 + 3/2*J2*(a/r)^2 * (3 - 5*(z/r)^2))
    // (z/r) = sin(地心緯度)。J4 以降の打ち切り誤差は ~1e-5 g。
    WGS84 wgs84;
    const double r = position_ECEF.norm();
    const double sin_gc_lat_sq = std::pow(position_ECEF(2) / r, 2);
    const double j2_term = 1.5 * wgs84.J2 * std::pow(wgs84.a / r, 2);
    const double newton = -wgs84.GM / (r * r * r);

    Eigen::Vector3d gravity_ECEF;
    gravity_ECEF(0) = newton * position_ECEF(0) * (1.0 + j2_term * (1.0 - 5.0 * sin_gc_lat_sq));
    gravity_ECEF(1) = newton * position_ECEF(1) * (1.0 + j2_term * (1.0 - 5.0 * sin_gc_lat_sq));
    gravity_ECEF(2) = newton * position_ECEF(2) * (1.0 + j2_term * (3.0 - 5.0 * sin_gc_lat_sq));
    return gravity_ECEF;
};
