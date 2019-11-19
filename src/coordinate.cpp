// ******************************************************
// Project Name    : ForRocket
// File Name       : coordinate.cpp
// Creation Date   : 2019/10/27
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "coordinate.hpp"

#include <cmath>


void forrocket::Coordinate::setWind2Body(const double alpha, const double beta) {
    dcm.wind2body << std::cos(alpha) * std::cos(beta), std::cos(alpha) * std::sin(beta), -std::sin(alpha),
                    -std::sin(beta)                  , std::cos(beta)                  ,              0.0,
                    std::sin(alpha) * std::cos(beta) , std::sin(alpha) * std::sin(beta), std::cos(alpha);
    dcm.body2wind = dcm.wind2body.transpose();
};


void forrocket::Coordinate::setNED2Body(const Eigen::Vector3d euler_angle) {
    double azi = euler_angle(0);
    double elv = euler_angle(1);
    double rol = euler_angle(2);
    dcm.NED2body << cos(azi) * cos(elv)                                  ,  sin(azi) * cos(elv)                                 , -sin(elv),
                    -sin(azi) * cos(rol) + cos(azi) * sin(elv) * sin(rol),  cos(azi) * cos(rol) + sin(azi) * sin(elv) * sin(rol),  cos(elv) * sin(rol),
                    sin(azi) * sin(rol) + cos(azi) * sin(elv) * cos(rol) , -cos(azi) * sin(rol) + sin(azi) * sin(elv) * cos(rol),  cos(elv) * cos(rol);
    dcm.body2NED = dcm.NED2body.transpose();
};

void forrocket::Coordinate::setNED2Body(const Eigen::Vector4d quat) {
    double q0 = quat(0);
    double q1 = quat(1);
    double q2 = quat(2);
    double q3 = quat(3);
    dcm.NED2body << q0*q0 - q1*q1 - q2*q2 + q3*q3, 2.0 * (q0 * q1 + q2 * q3)    , 2.0 * (q0 * q2 - q1 * q3),
                    2.0 * (q0 * q1 - q2 * q3)    , q1*q1 - q0*q0 - q2*q2 + q3*q3, 2.0 * (q1 * q2 + q0 * q3),
                    2.0 * (q0 * q2 + q1 * q3)    , 2.0 * (q1 * q2 - q0 * q3)    , q2*q2 - q0*q0 - q1*q1 + q3*q3;
    dcm.body2NED = dcm.NED2body.transpose();
};


void forrocket::Coordinate::setECI2ECEF(const double epoch_time) {
    double xi = wgs.omega * epoc_time;
    dcm.ECI2ECEF << std::cos(xi) , std::sin(xi), 0.0,
                    -std::sin(xi), std::cos(xi), 0.0,
                    0.0          ,          0.0, 1.0;
    dcm.ECEF2ECI = dcm.ECI2ECEF.transpose();
};


void forrocket::Coordinate::setECEF2NED(const Eigen::Vector3d pos_LLH_init) {
    double lat = pos_LLH_init(0) * 3.14159265 / 180.0;
    double lon = pos_LLH_init(1) * 3.14159265 / 180.0;

    dcm.ECEF2NED << -std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat),
                    -std::sin(lon)                 ,                  std::cos(lon),             0,
                    -std::cos(lat) * std::cos(lon) , -std::cos(lat) * std::sin(lon), -std::sin(lat);
    dcm.NED2ECEF = dcm.ECEF2NED.transpose();
};


Eigen::Vector4d forrocket::Coordinate::Quaternion(const Eigen::Vector3d euler_angle) {
    setNED2Body(euler_angle);
    Eigen::Vector4d q;
    q(0) = 0.5 * std::sqrt(1.0 + dcm.NED2body(0, 0) - dcm.NED2body(1, 1) - dcm.NED2body(2, 2));
    q(1) = 0.5 * std::sqrt(1.0 - dcm.NED2body(0, 0) + dcm.NED2body(1, 1) - dcm.NED2body(2, 2));
    q(2) = 0.5 * std::sqrt(1.0 - dcm.NED2body(0, 0) - dcm.NED2body(1, 1) + dcm.NED2body(2, 2));
    q(3) = 0.5 * std::sqrt(1.0 + dcm.NED2body(0, 0) + dcm.NED2body(1, 1) + dcm.NED2body(2, 2));

    Eigen::VectorXd::Index index_quat_max;
    double quat_max = q.maxCoeff(&index_quat_max);
    switch(index_quat_max) {
        case 0:
            q(0) = 0.5 * sqrt(1.0 + dcm.NED2body(0, 0) - dcm.NED2body(1,1) - dcm.NED2body(2,2));
            q(1) = (dcm.NED2body(0, 1) + dcm.NED2body(1, 0)) / (4.0 * q(0));
            q(2) = (dcm.NED2body(2, 0) + dcm.NED2body(0, 2)) / (4.0 * q(0));
            q(3) = (dcm.NED2body(1, 2) - dcm.NED2body(2, 1)) / (4.0 * q(0));
            break;
        case 1:
            q(1) = 0.5 * sqrt(1.0 - dcm.NED2body(0, 0) + dcm.NED2body(1,1) - dcm.NED2body(2,2));
            q(0) = (dcm.NED2body(0, 1) + dcm.NED2body(1, 0)) / (4.0 * q(1));
            q(2) = (dcm.NED2body(1, 2) + dcm.NED2body(2, 1)) / (4.0 * q(1));
            q(3) = (dcm.NED2body(2, 0) - dcm.NED2body(0, 2)) / (4.0 * q(1));
            break;
        case 2:
            q(2) = 0.5 * sqrt(1.0 - dcm.NED2body(0, 0) - dcm.NED2body(1,1) + dcm.NED2body(2,2));
            q(0) = (dcm.NED2body(2, 0) + dcm.NED2body(0, 2)) / (4.0 * q(2));
            q(1) = (dcm.NED2body(1, 2) + dcm.NED2body(2, 1)) / (4.0 * q(2));
            q(3) = (dcm.NED2body(0, 1) - dcm.NED2body(1, 0)) / (4.0 * q(2));
            break;
        case 3:
            q(3) = 0.5 * sqrt(1.0 + dcm.NED2body(0, 0) + dcm.NED2body(1,1) + dcm.NED2body(2,2));
            q(0) = (dcm.NED2body(1, 2) - dcm.NED2body(2, 1)) / (4.0 * q(3));
            q(1) = (dcm.NED2body(2, 0) - dcm.NED2body(0, 2)) / (4.0 * q(3));
            q(2) = (dcm.NED2body(0, 1) - dcm.NED2body(1, 0)) / (4.0 * q(3));  
            break;
        default:
            break;
    }
    q.normalize();
    return q;
};


Eigen::Vector3d forrocket::Coordinate::EulerAngle() {
    double azimuth = std::atan2(dcm.NED2body(0, 1), dcm.NED2body(0, 0));
    double elevation = -std::asin(dcm.NED2body(0, 2));
    double roll = std::atan2(dcm.NED2body(1, 2), dcm.NED2body(2, 2));
    Eigen::Vector3d euler;
    euler << azimuth, elevation, roll;
    return euler;
};




















