// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_6dof_aero.cpp
// Creation Date   : 2019/10/20
//
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "dynamics_6dof_aero.hpp"

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "boost/numeric/odeint.hpp"

#include "environment/air.hpp"
#include "coordinate.hpp"

namespace odeint = boost::numeric::odeint;

forrocket::Dynamics6dofAero::Dynamics6dofAero(Rocket* rocket) {
    p_rocket = rocket;
};

void forrocket::Dynamics6dofAero::operator()(const state& x, state& dx, const double t) {
    // update flight infomation
    p_rocket->position.ECI[0] = x[0]; p_rocket->position.ECI[1] = x[1]; p_rocket->position.ECI[2] = x[2];
    p_rocket->velocity.ECI[0] = x[3]; p_rocket->velocity.ECI[1] = x[4]; p_rocket->velocity.ECI[2] = x[5];
    p_rocket->quaternion[0] = x[6]; p_rocket->quaternion[1] = x[7]; p_rocket->quaternion[2] = x[8]; p_rocket->quaternion[3] = x[9]; 
    p_rocket->omega[0] = x[10]; p_rocket->omega[1] = x[11]; p_rocket->omega[2] = x[12];
    p_rocket->mass.propellant = x[13];

    Coordinate coordinate;

    double altitude;

    forrocket::EnvironmentAir env_air(altitude);

    
    

    dx[0] = p_rocket->velocity.ECI[0];  // vel_ECI => pos_ECI
    dx[1] = p_rocket->velocity.ECI[1];  // 
    dx[2] = p_rocket->velocity.ECI[2];  // 
    dx[3] = p_rocket->acceleration.ECI[0];  // acc_ECI => vel_ECI
    dx[4] = p_rocket->acceleration.ECI[1];  // 
    dx[5] = p_rocket->acceleration.ECI[2];  // 
    dx[6] = p_rocket->quaternion_dot[0];  // quatdot => quat
    dx[7] = p_rocket->quaternion_dot[1];  // 
    dx[8] = p_rocket->quaternion_dot[2];  // 
    dx[9] = p_rocket->quaternion_dot[3];  // 
    dx[10] = p_rocket->omega_dot[0];  // omegadot => omega
    dx[11] = p_rocket->omega_dot[1];  // 
    dx[12] = p_rocket->omega_dot[2];  // 
    dx[13] = -p_rocket->engine.mdot_prop;  // massdot => mass_prop
};