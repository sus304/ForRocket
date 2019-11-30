// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket.cpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket.hpp"

#include <cmath>

forrocket::Rocket::Rocket() {
    length_thrust = 0.0;
    diameter = 0.0;
    area = 0.0;
    length = 0.0;
    length_CG = 0.0;
    length_CP = 0.0;
    inertia_tensor << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    CA = 0.0;
    CNa = 0.0;
    Clp = 0.0;
    Cmq = 0.0;
    Cnr = 0.0;

    cant_angle_fin = 0.0;
    CNa_fin = 0.0;
    radius_CP_fin = 0.0;

    dynamic_pressure = 0.0;

    force_thrust << 0.0, 0.0, 0.0;
    force_aero << 0.0, 0.0, 0.0;

    quaternion << 0.0, 0.0, 0.0, 0.0;
    quaternion_dot << 0.0, 0.0, 0.0, 0.0;
    angular_velocity << 0.0, 0.0, 0.0;
    angular_acceleration << 0.0, 0.0, 0.0;
    angle_of_attack = 0.0;
    sideslip_angle = 0.0;

    moment_gyro << 0.0, 0.0, 0.0;
    moment_thrust << 0.0, 0.0, 0.0;
    moment_aero_force << 0.0, 0.0, 0.0;
    moment_aero_dumping << 0.0, 0.0, 0.0;
    moment_jet_dumping << 0.0, 0.0, 0.0;
};


void forrocket::Rocket::InitializePosition(const DateTime datetime_UTC_init, const Eigen::Vector3d LLH) {
    position.Initialize(datetime_UTC_init, LLH);
};



void forrocket::Rocket::UpdateLengthCG(const double t) {


};


void forrocket::Rocket::UpdateLengthCP() {


};


void forrocket::Rocket::UpdateInertiaTensor(const double t) {

};


void forrocket::Rocket::UpdateEngine(const double t, const double air_pressure, const double air_pressure_sea_level) {
    if (mass.propellant <= 0.0) {
        engine.Cutoff();
    } else {
        engine.Update(t, air_pressure_sea_level, air_pressure);
    }
}


void forrocket::Rocket::UpdateAerodynamicsCoefficient() {
    // double CA;
    // double CNa;
    // double Clp;
    // double Cmq;
    // double Cnr;
};


void forrocket::Rocket::UpdateAttitudeFromProgramRate(const double t) {

};



