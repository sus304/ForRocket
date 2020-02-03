// ******************************************************
// Project Name    : ForRocket
// File Name       : flight_data_recorder.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "flight_data_recorder.hpp"

forrocket::FlightDataRecorder::FlightDataRecorder(Rocket* rocket) {
    p_rocket = rocket;
};


void forrocket::FlightDataRecorder::operator()(const DynamicsBase::state& x, DynamicsBase::state& dx, const double t) {
    countup_burn_time.push_back(p_rocket->burn_clock.countup_time);
    thrust.push_back(p_rocket->engine.thrust);
    mdot_prop.push_back(p_rocket->engine.mdot_prop);
    burning.push_back(p_rocket->engine.burning);
    mass_prop.push_back(p_rocket->mass.propellant);
    mass.push_back(p_rocket->mass.inert + p_rocket->mass.propellant);
    length_CG.push_back(p_rocket->length_CG);
    length_CP.push_back(p_rocket->length_CP);
    CA.push_back(p_rocket->CA);
    CNa.push_back(p_rocket->CNa);
    Clp.push_back(p_rocket->Clp);
    Cmq.push_back(p_rocket->Cmq);
    postion.push_back(p_rocket->position);
    velocity.push_back(p_rocket->velocity);
    dynamic_pressure.push_back(p_rocket->dynamic_pressure);
    acceleration.push_back(p_rocket->acceleration);
    force.push_back(p_rocket->force);
    attitude.push_back(p_rocket->attitude);
    quaternion_dot.push_back(p_rocket->quaternion_dot);
    angular_velocity.push_back(p_rocket->angular_velocity);
    angular_acceleration.push_back(p_rocket->angular_acceleration);
    angle_of_attack.push_back(p_rocket->angle_of_attack);
    sideslip_angle.push_back(p_rocket->sideslip_angle);
    moment.push_back(p_rocket->moment);
};
