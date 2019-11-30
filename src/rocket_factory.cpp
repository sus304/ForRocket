// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_factory.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket_factory.hpp"

#include <cmath>

const double pi = 3.14159265;

forrocket::Rocket* forrocket::RocketFactory::Create(const DateTime datetime) {
    Rocket* rocket = new Rocket();

    rocket->length_thrust = 0.3;
    rocket->diameter = 0.154;
    rocket->area = 0.25 * std::pow(rocket->diameter, 2) * pi;
    rocket->length = 2.0;
    rocket->mass.inert = 9.8;
    rocket->mass.propellant = 0.3;
    rocket->length_CG = 1.0;
    rocket->length_CP = 0.5;
    rocket->inertia_tensor << 0.3, 0, 0, 0, 3.0, 0, 0, 0, 3.0;
    rocket->CA = 0.5;
    rocket->CNa = 9.0;
    rocket->Clp = 0.02;
    rocket->Cmq = 0.4;
    rocket->Cnr = rocket->Cmq;
    rocket->cant_angle_fin = 0.0;
    rocket->CNa_fin = 8.0;
    rocket->radius_CP_fin = 0.17;
    rocket->
    rocket->
    rocket->
    rocket->
    rocket->
    rocket->
    rocket->
    rocket->
    rocket->
    rocket->
    rocket->

    return rocket;
};


            Engine engine;



            Position position;
            Velocity velocity;
            double dynamic_pressure;
            Acceleration acceleration;

            Eigen::Vector3d force_thrust;
            Eigen::Vector3d force_aero;

            Eigen::Vector4d quaternion;
            Eigen::Vector4d quaternion_dot;
            Eigen::Vector3d angular_velocity;
            Eigen::Vector3d angular_acceleration;
            double angle_of_attack;
            double sideslip_angle;

            Eigen::Vector3d moment_gyro;
            Eigen::Vector3d moment_thrust;
            Eigen::Vector3d moment_aero_force;
            Eigen::Vector3d moment_aero_dumping;
            Eigen::Vector3d moment_jet_dumping;

            void UpdateLengthCG(const double t);
            void UpdateLengthCP();
            void UpdateInertiaTensor(const double t);
            void UpdateEngine(const double t, const double air_pressure, const double air_pressure_sea_level);
            void UpdateAerodynamicsCoefficient();
            void UpdateAttitudeFromProgramRate(const double t);