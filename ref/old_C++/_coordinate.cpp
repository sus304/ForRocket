// ******************************************************
// Project Name    : ForRocket
// File Name       : coordinate.cpp
// Creation Date   : 2018/11/04
//
// Copyright © 2018 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "coordinate.hpp"

Eigen::Matrix3d ForRocket::DCM_WIND2BODY(double alpha, double beta)
{
    Eigen::Matrix3d DCM;
    DCM << cos(alpha) * cos(beta), cos(alpha) * sin(beta), -sin(alpha),
          -sin(beta)             , cos(beta)             , 0.0,
           sin(alpha) * cos(beta), sin(alpha) * sin(beta), cos(alpha);
    return DCM;
};

Eigen::Matrix3d ForRocket::DCM_NED2BODY_euler(double azimuth_deg, double elevation_deg, double roll_deg)
{
    double azi = deg2rad(azimuth_deg);
    double elv = deg2rad(elevation_deg);
    double rol = deg2rad(roll_deg);
    Eigen::Matrix3d DCM;
    DCM << cos(azi) * cos(elv)                                 ,  sin(azi) * cos(elv)                                 , -sin(elv),
          -sin(azi) * cos(rol) + cos(azi) * sin(elv) * sin(rol),  cos(azi) * cos(rol) + sin(azi) * sin(elv) * sin(rol),  cos(elv) * sin(rol),
           sin(azi) * sin(rol) + cos(azi) * sin(elv) * cos(rol), -cos(azi) * sin(rol) + sin(azi) * sin(elv) * cos(rol),  cos(elv) * cos(rol);
    return DCM;    
};

Eigen::Vector4d ForRocket::quat_normalize(Eigen::Vector4d quat)
{
    return quat.normalized();
};

Eigen::Matrix3d ForRocket::DCM_NED2BODY_quat(Eigen::Vector4d quat)
{
    double q0 = quat(0);
    double q1 = quat(1);
    double q2 = quat(2);
    double q3 = quat(3);
    Eigen::Matrix3d DCM;
    DCM << q0*q0 - q1*q1 - q2*q2 + q3*q3, 2.0 * (q0 * q1 + q2 * q3)    , 2.0 * (q0 * q2 - q1 * q3),
           2.0 * (q0 * q1 - q2 * q3)    , q1*q1 - q0*q0 - q2*q2 + q3*q3, 2.0 * (q1 * q2 + q0 * q3),
           2.0 * (q0 * q2 + q1 * q3)    , 2.0 * (q1 * q2 - q0 * q3)    , q2*q2 - q0*q0 - q1*q1 + q3*q3;
    return DCM;
};


Eigen::Vector4d euler2quat(double azimuth_deg, double elevation_deg, double roll_deg)
{
    double azi = deg2rad(azimuth_deg);
    double elv = deg2rad(elevation_deg);
    double roll = deg2rad(roll_deg);

    Eigen::Matrix3d DCM;
    Eigen::Vector4d q;
    DCM = ForRocket::DCM_NED2BODY_euler(azimuth_deg, elevation_deg, roll_deg);
    q(0) = 0.5 * sqrt(1.0 + DCM(0,0) - DCM(1,1) - DCM(2,2));
    q(1) = 0.5 * sqrt(1.0 - DCM(0,0) + DCM(1,1) - DCM(2,2));
    q(2) = 0.5 * sqrt(1.0 - DCM(0,0) - DCM(1,1) + DCM(2,2));
    q(3) = 0.5 * sqrt(1.0 + DCM(0,0) + DCM(1,1) + DCM(2,2));

    Eigen::VectorXd::Index quat_max_index;
    double quat_max = q.maxCoeff(&quat_max_index);
    switch (quat_max_index)
    {
        case 0:
            q(0) = 0.5 * sqrt(1.0 + DCM(0, 0) - DCM(1,1) - DCM(2,2));
            q(1) = (DCM(1, 0) + DCM(0, 1)) / (4.0 * q(0));
            q(2) = (DCM(0, 2) + DCM(2, 0)) / (4.0 * q(0));
            q(3) = (DCM(2, 1) - DCM(1, 2)) / (4.0 * q(0));
            break;
        case 1:
            q(1) = 0.5 * sqrt(1.0 - DCM(0, 0) + DCM(1,1) - DCM(2,2));
            q(0) = (DCM(1, 0) + DCM(0, 1)) / (4.0 * q(1));
            q(2) = (DCM(2, 1) + DCM(1, 2)) / (4.0 * q(1));
            q(3) = (DCM(0, 2) - DCM(2, 0)) / (4.0 * q(1));
        case 2:
            q(2) = 0.5 * sqrt(1.0 - DCM(0, 0) - DCM(1,1) + DCM(2,2));
            q(0) = (DCM(0, 1) + DCM(2, 0)) / (4.0 * q(2));
            q(1) = (DCM(2, 1) + DCM(1, 2)) / (4.0 * q(2));
            q(3) = (DCM(1, 0) - DCM(0, 1)) / (4.0 * q(2));
        case 4:
            q(3) = 0.5 * sqrt(1.0 + DCM(0, 0) + DCM(1,1) + DCM(2,2));
            q(0) = (DCM(1, 2) - DCM(2, 1)) / (4.0 * q(3));
            q(1) = (DCM(2, 0) - DCM(0, 2)) / (4.0 * q(3));
            q(2) = (DCM(0, 1) - DCM(1, 0)) / (4.0 * q(3));  
        default:
            break;
    }
    q.normalize();

    return q;
};


Eigen::Vector3d ForRocket::quat2euler(Eigen::Matrix3d DCM_NED2BODY)
{
    Eigen::Vector3d attitude;
    attitude(0) = rad2deg(atan2(DCM_NED2BODY(1, 0), DCM_NED2BODY(0, 0)));
    attitude(1) = rad2deg(-asin(DCM_NED2BODY(2, 0)));
    attitude(2) = rad2deg(atan2(DCM_NED2BODY(2, 1), DCM_NED2BODY(2, 2)));
    return attitude;

    // TODO: link azimuth and roll
};


Eigen::Matrix3d DCM_ECI2ECEF(double t_sec)
{
    double omega_earth = 7.2921151e-5;  // [rad/s] for WGS84
    double xi = omega_earth * t_sec;
    Eigen::Matrix3d DCM;
    DCM << cos(xi), sin(xi), 0.0,
           -sin(xi), cos(xi), 0.0,
           0.0, 0.0, 1.0;
    return DCM;
};


Eigen::Matrix3d DCM_ECEF2NED(Eigen::Vector3d pos_LLH_init)
{
    // input: deg, deg, m
    double lat = deg2rad(pos_LLH_init(0));
    double lon = deg2rad(pos_LLH_init(1));
    Eigen::Matrix3d DCM;
    DCM << -sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat),
           -sin(lon), cos(lon), 0.0,
           -cos(lat) * cos(lon), -cos(lat) * sin(lon), -sin(lat);
    return DCM;

    // NOTE: from MATLAB
};


Eigen::Vector3d vel_ECI2ECEF(Eigen::Vector3d vel_ECI, Eigen::Matrix3d DCM_ECI2ECEF, Eigen::Vector3d pos_ECI)
{
    double omega_earth = 7.2921151e-5;  // [rad/s] for WGS84
    Eigen::Matrix3d omega;
    omega << 0.0, -omega_earth, 0.0,
           omega_earth, 0.0, 0.0,
           0.0, 0.0, 0.0;
    Eigen::Vector3d vel;
    vel = DCM_ECI2ECEF * vel_ECI - omega * pos_ECI;
    return vel;
};


Eigen::Vector3d vel_ECEF2ECI(Eigen::Vector3d vel_ECEF, Eigen::Matrix3d DCM_ECI2ECEF, Eigen::Vector3d pos_ECI)
{
    double omega_earth = 7.2921151e-5;  // [rad/s] for WGS84
    Eigen::Matrix3d omega;
    omega << 0.0, -omega_earth, 0.0,
           omega_earth, 0.0, 0.0,
           0.0, 0.0, 0.0;
    Eigen::Vector3d vel;
    vel = DCM_ECI2ECEF * vel_ECEF + omega * pos_ECI;
    return vel;
};

