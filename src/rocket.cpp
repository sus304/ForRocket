// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket.cpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket.hpp"

#include <cmath>

void forrocket::Rocket::UpdatePosition() {
    // position.ECEF;
    // position.LLH;
    // position.NED;

};


void forrocket::Rocket::UpdateVelocity() {
    // velocity.ECEF;
    // velocity.NED;
    // velocity.body;
    // velocity.air_body;
    // velocity.mach_number;
    // double dynamic_pressure;

};


void forrocket::Rocket::UpdateLengthCG(const double t) {


};


void forrocket::Rocket::UpdateLengthCP(const double mach) {


};


void forrocket::Rocket::UpdateEngine(const double t, const double air_pressure) {
    forrocket::EnvironmentAir env_air_sea_level(0.0);
    if (t > engine.burn_duration || mass.propellant <= 0.0) {
        engine.Cutoff();
    }
    else {
        engine.Update(t, env_air_sea_level.pressure, air_pressure);
    }
}


void forrocket::Rocket::UpdateAerodynamicsCoefficient(const double mach) {
    // double CA;
    // double CNa;
    // double Clp;
    // double Cmq;
    // double Cnr;
};


void forrocket::Rocket::UpdateAoA(Eigen::Vector3d velocity_air_body) {
    double abs = velocity_air_body.norm();
    if (std::abs(velocity_air_body(0)) <= 0.0) angle_of_attack = 0.0;
    else angle_of_attack = std::atan2(velocity_air_body(2), velocity_air_body(0));

    if (abs <= 0.0) sideslip_angle = 0.0;
    else sideslip_angle = std::asin(-velocity_air_body(1) / abs);
};


void forrocket::Rocket::UpdateAttitudeFromProgramRate(const double t) {

};



