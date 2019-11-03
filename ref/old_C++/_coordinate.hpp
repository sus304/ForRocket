// ******************************************************
// Project Name    : ForRocket
// File Name       : coordinate.hpp
// Creation Date   : 2018/11/04
//
// Copyright © 2018 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef coordinate.hpp
#define coordinate.hpp

#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>

#include "../lib/Eigen/Core"


namespace ForRocket
{
    Eigen::Matrix3d ForRocket::DCM_WIND2BODY(double alpha, double beta);
    Eigen::Matrix3d ForRocket::DCM_NED2BODY_euler(double azimuth_deg, double elevation_deg, double roll_deg);
    Eigen::Vector4d ForRocket::quat_normalize(Eigen::Vector4d quat);
    Eigen::Matrix3d ForRocket::DCM_NED2BODY_quat(Eigen::Vector4d quat);
    Eigen::Vector4d ForRocket::euler2quat(double azimuth_deg, double elevation_deg, double roll_deg);
    Eigen::Vector3d ForRocket::quat2euler(Eigen::Matrix3d DCM_NED2BODY);
    Eigen::Matrix3d ForRocket::DCM_ECI2ECEF(double t_sec);
    Eigen::Matrix3d ForRocket::DCM_ECEF2NED(Eigen::Vector3d pos_LLH_init);
    Eigen::Vector3d ForRocket::vel_ECI2ECEF(Eigen::Vector3d vel_ECI, Eigen::Matrix3d DCM_ECI2ECEF, Eigen::Vector3d pos_ECI);
    Eigen::Vector3d ForRocket::vel_ECEF2ECI(Eigen::Vector3d vel_ECEF, Eigen::Matrix3d DCM_ECI2ECEF, Eigen::Vector3d pos_ECI);
}

inline double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}
inline double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

#endif