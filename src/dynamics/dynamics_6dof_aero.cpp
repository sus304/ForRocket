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

#include "environment/coordinate.hpp"
#include "environment/air.hpp"

namespace odeint = boost::numeric::odeint;

forrocket::Dynamics6dofAero::Dynamics6dofAero(Rocket* rocket, SequenceClock* clock, EnvironmentWind* wind) {
    p_rocket = rocket;
    p_clock = clock;
    p_wind = wind;
};


void forrocket::Dynamics6dofAero::operator()(const state& x, state& dx, const double t) {
    Coordinate coordinate;

    // Countup Time
    p_clock->SyncSolverTime(t);

    // Update Flight Infomation
    coordinate.setECI2ECEF(p_clock->greenwich_sidereal_time);
    p_rocket->position.Update(coordinate, Eigen::Map<Eigen::Vector3d>(std::vector<double>(x.begin()+0, x.begin()+3).data()));
    coordinate.setECEF2NED(p_rocket->position.LLH);
    p_rocket->velocity.Update(coordinate, Eigen::Map<Eigen::Vector3d>(std::vector<double>(x.begin()+3, x.begin()+6).data()), p_rocket->position.ECI);

    p_rocket->quaternion = Eigen::Map<Eigen::Vector4d>(std::vector<double>(x.begin()+6, x.begin()+10).data());
    p_rocket->quaternion.normalize();
    coordinate.setNED2Body(p_rocket->quaternion);

    p_rocket->angular_velocity = Eigen::Map<Eigen::Vector3d>(std::vector<double>(x.begin()+10, x.begin()+13).data());

    p_rocket->mass.propellant = x[13];


    // Update Environment
    double altitude = p_rocket->position.LLH[2];
    EnvironmentAir air(altitude);
    EnvironmentAir air_sea_level(0.0);
    p_wind->Update(altitude);
    Eigen::Vector3d gravity_NED(0.0, 0.0, gravity(altitude));

    // Update Airspeed
    p_rocket->velocity.air_body = coordinate.dcm.NED2body * (p_rocket->velocity.NED - p_wind->NED);
    p_rocket->dynamic_pressure = 0.5 * air.density * std::pow(p_rocket->velocity.air_body.norm(), 2);
    p_rocket->velocity.mach_number = p_rocket->velocity.air_body.norm() / air.speed_of_sound;

    // Update Flight Infomation 2
    p_rocket->engine.Update(p_clock->countup_time, air.pressure, air_sea_level.pressure);
    p_rocket->UpdateLengthCG(p_clock->countup_time);
    p_rocket->UpdateAerodynamicsParameter(p_rocket->velocity.mach_number);

    // Calculate AoA
    p_rocket->angle_of_attack = std::atan2(p_rocket->velocity.air_body[2], p_rocket->velocity.air_body[0]);
    if (p_rocket->velocity.air_body.norm() <= 0.0) {
        p_rocket->sideslip_angle = 0.0;
    } else {
        p_rocket->sideslip_angle = std::asin(-p_rocket->velocity.air_body[1] / p_rocket->velocity.air_body.norm());
    }


    // Calculate Force
    p_rocket->force.thrust = ThrustBodyCoordinate();
    p_rocket->force.aero = AeroForceBodyCoordinate();
    p_rocket->force.gravity = coordinate.dcm.NED2body * gravity_NED * (p_rocket->mass.inert + p_rocket->mass.propellant);

    // Calculate Acceleration
    p_rocket->acceleration.body = p_rocket->force.Sum() / (p_rocket->mass.inert + p_rocket->mass.propellant);
    p_rocket->acceleration.ECI = coordinate.dcm.ECEF2ECI * (coordinate.dcm.NED2ECEF * (coordinate.dcm.body2NED * p_rocket->acceleration.body));

    // Calculate Moment
    p_rocket->moment.gyro = GyroEffectMoment();
    p_rocket->moment.thrust = ThrustMoment();
    p_rocket->moment.aero_force = AeroForceMoment();
    p_rocket->moment.aero_dumping = AeroDampingMoment();
    p_rocket->moment.jet_dumping = JetDampingMoment();
    
    // Calculate Angle Velocity
    p_rocket->angular_acceleration = p_rocket->inertia_tensor.inverse() * p_rocket->moment.Sum();

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
