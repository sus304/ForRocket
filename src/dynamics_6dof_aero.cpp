// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_6dof_aero.cpp
// Creation Date   : 2019/10/20
//
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "dynamics_6dof_aero.hpp"

#include <cmath>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "boost/numeric/odeint.hpp"

#include "coordinate.hpp"
#include "environment/air.hpp*

namespace odeint = boost::numeric::odeint;

forrocket::Dynamics6dofAero::Dynamics6dofAero(Rocket* rocket, SequenceClock* clock, EnvironmentWind* wind) {
    p_rocket = rocket;
    p_clock = clock;
    p_wind = wind;
};


void forrocket::Dynamics6dofAero::operator()(const state& x, state& dx, const double t) {
    // Update Flight Infomation
    p_rocket->position.ECI[0] = x[0]; p_rocket->position.ECI[1] = x[1]; p_rocket->position.ECI[2] = x[2];
    p_rocket->velocity.ECI[0] = x[3]; p_rocket->velocity.ECI[1] = x[4]; p_rocket->velocity.ECI[2] = x[5];
    p_rocket->quaternion[0] = x[6]; p_rocket->quaternion[1] = x[7]; p_rocket->quaternion[2] = x[8]; p_rocket->quaternion[3] = x[9]; 
    p_rocket->angular_velocity[0] = x[10]; p_rocket->angular_velocity[1] = x[11]; p_rocket->angular_velocity[2] = x[12];
    p_rocket->mass.propellant = x[13];
    p_rocket->quaternion.normalize();

    // Countup Time
    p_clock->SyncSolverTime(t);

    // Coordinate Transform(Position)
    Coordinate coordinate;
    coordinate.setECI2ECEF(p_clock->greenwich_sidereal_time);
    p_rocket->position.ECEF = coordinate.dcm.ECI2ECEF * p_rocket->position.ECI;
    p_rocket->position.LLH = coordinate.ECEF2LLH(p_rocket->position.ECEF);
    coordinate.setECEF2NED(p_rocket->position.LLH);
    coordinate.setNED2Body(p_rocket->quaternion);

    // Coordinate Transform(Velocity)
    p_rocket->velocity.ECEF = coordinate.dcm.ECI2ECEF * p_rocket->velocity.ECI - coordinate.dcm.EarthRotate * p_rocket->position.ECI;
    p_rocket->velocity.NED = coordinate.dcm.ECEF2NED * p_rocket->velocity.ECEF;

    // Update Environment
    double altitude = p_rocket->position.LLH[2];
    EnvironmentAir air(altitude);
    EnvironmentAir air_sea_level(0.0);
    p_wind->Update(altitude);
    Eigen::Vector3d gravity_NED(0.0, 0.0, gravity(altitude));
    // double g0 = gravity(0.0);

    // Update Airspeed
    p_rocket->velocity.air_body = coordinate.dcm.NED2body * (p_rocket->velocity.NED - p_wind->NED);
    p_rocket->dynamic_pressure = 0.5 * air.density * std::pow(p_rocket->velocity.air_body.norm(), 2);
    p_rocket->velocity.mach_number = p_rocket->velocity.air_body.norm() / air.speed_of_sound;

    // Update Function
    p_rocket->UpdateEngine(p_clock->countup_time, air.pressure, air_sea_level.pressure);
    p_rocket->UpdateAerodynamicsCoefficient();

    // Calculate AoA
    p_rocket->angle_of_attack = std::atan2(p_rocket->velocity.air_body[2], p_rocket->velocity.air_body[0]);
    // if (std::abs(p_rocket->velocity.air_body[0]) <= 0.0) {
    //     p_rocket->angle_of_attack = 90.0 * 3.14159265 / 180.0;
    // } else {
    //     p_rocket->angle_of_attack = std::atan2(p_rocket->velocity.air_body[2], p_rocket->velocity.air_body[0]);
    // }
    if (p_rocket->velocity.air_body.norm() <= 0.0) {
        p_rocket->sideslip_angle = 0.0;
    } else {
        p_rocket->sideslip_angle = std::asin(-p_rocket->velocity.air_body[1] / p_rocket->velocity.air_body.norm());
    }


    // Calculate Force
    p_rocket->force_thrust = ThrustBodyCoordinate();
    p_rocket->force_aero = AeroForceBodyCoordinate();

    // Calculate Acceleration
    Eigen::Vector3d force = p_rocket->force_thrust + p_rocket->force_aero + gravity_NED * (p_rocket->mass.inert + p_rocket->mass.propellant);
    p_rocket->acceleration.body = force / (p_rocket->mass.inert + p_rocket->mass.propellant);
    p_rocket->acceleration.ECI = coordinate.dcm.ECEF2ECI * (coordinate.dcm.NED2ECEF * (coordinate.dcm.body2NED * p_rocket->acceleration.body));

    // Update Center of Gravity, Center of Pressure, Inertia Moment
    p_rocket->UpdateLengthCG(p_clock->countup_time);
    p_rocket->UpdateLengthCP();
    p_rocket->UpdateInertiaTensor(p_clock->countup_time);

    // Calculate Moment
    p_rocket->moment_gyro = GyroEffectMoment();
    p_rocket->moment_thrust = ThrustMoment();
    p_rocket->moment_aero_force = AeroForceMoment();
    p_rocket->moment_aero_dumping = AeroDampingMoment();
    p_rocket->moment_jet_dumping = JetDampingMoment();
    
    // Calculate Angle Velocity
    Eigen::Vector3d moment = p_rocket->moment_gyro + p_rocket->moment_thrust + p_rocket->moment_aero_force + p_rocket->moment_aero_dumping + p_rocket->moment_jet_dumping;
    p_rocket->angular_acceleration = p_rocket->inertia_tensor.inverse() * moment;

    // Calculate Quaternion
    p_rocket->quaternion_dot = 0.5 * QuaternionDiff();


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
    dx[10] = p_rocket->angular_acceleration[0];  // angular_acceleration => angular_velocity
    dx[11] = p_rocket->angular_acceleration[1];  // 
    dx[12] = p_rocket->angular_acceleration[2];  // 
    dx[13] = -p_rocket->engine.mdot_prop;  // massdot => mass_prop
};