// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_3dof_parachute.cpp
// Creation Date   : 2020/02/01
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "dynamics_3dof_parachute.hpp"

#include <cmath>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "boost/numeric/odeint.hpp"

#include "environment/coordinate.hpp"
#include "environment/air.hpp"
#include "environment/gravity.hpp"

forrocket::Dynamics3dofParachute::Dynamics3dofParachute(Rocket* rocket, SequenceClock* clock, EnvironmentWind* wind) {
    p_rocket = rocket;
    p_clock = clock;
    p_wind = wind;
};


void forrocket::Dynamics3dofParachute::operator()(const state& x, state& dx, const double t) {
    Coordinate coordinate;

    // Countup Time
    p_clock->SyncSolverTime(t);
    p_rocket->burn_clock.SyncSolverTime(t);

    // Update Flight Infomation
    coordinate.setECI2ECEF(t);

    p_rocket->position.ECI = Eigen::Map<Eigen::Vector3d>(std::vector<double>(x.begin()+0, x.begin()+3).data());
    p_rocket->position.ECEF = coordinate.dcm.ECI2ECEF * p_rocket->position.ECI;
    p_rocket->position.LLH = coordinate.ECEF2LLH(p_rocket->position.ECEF);

    coordinate.setECEF2NED(p_rocket->position.LLH);

    p_rocket->velocity.ECI = Eigen::Map<Eigen::Vector3d>(std::vector<double>(x.begin()+3, x.begin()+6).data());
    p_rocket->velocity.ECEF = coordinate.dcm.ECI2ECEF * (p_rocket->velocity.ECI - coordinate.dcm.EarthRotate * p_rocket->position.ECI);
    p_rocket->velocity.NED = coordinate.dcm.ECEF2NED * p_rocket->velocity.ECEF;
    double decent_velocity = p_rocket->velocity.NED(2);

    // Update Environment
    double altitude = p_rocket->position.LLH[2];
    EnvironmentAir air(altitude);
    EnvironmentAir air_sea_level(0.0);
    Eigen::Vector3d gravity_NED(0.0, 0.0, gravity(altitude));

    Eigen::Vector3d drag_NED(0.0, 0.0, -0.5 * air.density * decent_velocity * abs(decent_velocity) * p_rocket->CdS_parachute);
    Eigen::Vector3d acceleration_NED = drag_NED / (p_rocket->mass.Sum()) + gravity_NED;
    p_rocket->acceleration.ECI = coordinate.dcm.ECEF2ECI * (coordinate.dcm.NED2ECEF * acceleration_NED);

    Eigen::Vector3d wind_NED = p_wind->getNED(altitude);
    p_rocket->velocity.NED += wind_NED;
    p_rocket->velocity.ECI = coordinate.dcm.ECEF2ECI * (coordinate.dcm.NED2ECEF * p_rocket->velocity.NED) + coordinate.dcm.EarthRotate * p_rocket->position.ECI;

    dx[0] = p_rocket->velocity.ECI[0];  // vel_ECI => pos_ECI
    dx[1] = p_rocket->velocity.ECI[1];  // 
    dx[2] = p_rocket->velocity.ECI[2];  // 
    dx[3] = p_rocket->acceleration.ECI[0];  // acc_ECI => vel_ECI
    dx[4] = p_rocket->acceleration.ECI[1];  // 
    dx[5] = p_rocket->acceleration.ECI[2];  // 
    dx[6] = 0.0;  // quatdot => quat
    dx[7] = 0.0;  // 
    dx[8] = 0.0;  // 
    dx[9] = 0.0;  // 
    dx[10] = 0.0;  // angular_acceleration => angular_velocity
    dx[11] = 0.0;  // 
    dx[12] = 0.0;  // 
    dx[13] = 0.0;  // massdot => mass_prop
};
