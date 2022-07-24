// ******************************************************
// Project Name    : ForRocket
// File Name       : engine.cpp
// Creation Date   : 2019/10/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "engine.hpp"

#include <numeric>

#ifdef DEBUG
#include <iostream>
#endif

void forrocket::Engine::Reset() {
    burning = false;

    thrust = 0.0;
    mdot_prop = 0.0;
    gimbal_angle_y_axis = 0.0;
    gimbal_angle_z_axis = 0.0;
    mis_alignment_angle_y_axis = 0.0;
    mis_alignment_angle_z_axis = 0.0;

    enable_gimbal = false;

    area_exit = 0.0;
    burn_duration = 0.0;
};

void forrocket::Engine::getTotalImpulse(const double thrust, const double burn_duration) {
    total_impulse = thrust * burn_duration;
};

void forrocket::Engine::getTotalImpulse(const std::vector<double> thrust, const double burn_duration) {
    total_impulse = std::accumulate(thrust.begin(), thrust.end(), 0.0) / thrust.size() * burn_duration;
};


forrocket::Engine::Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit) {
    Reset();
    enable_gimbal = false;

    this->area_exit = area_exit;

    this->burn_duration = burn_duration;
    thrust_vacuum_src = InterpolateParameter(thrust_const);
    mdot_prop_src = InterpolateParameter(mdot_prop_const);

    getTotalImpulse(thrust_const, burn_duration);
};


forrocket::Engine::Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis) {
    Reset();
    enable_gimbal = false;

    this->area_exit = area_exit;

    this->burn_duration = burn_duration;
    thrust_vacuum_src = InterpolateParameter(thrust_const);
    mdot_prop_src = InterpolateParameter(mdot_prop_const);
    this->mis_alignment_angle_y_axis = mis_alignment_angle_y_axis;
    this->mis_alignment_angle_z_axis = mis_alignment_angle_z_axis;

    getTotalImpulse(thrust_const, burn_duration);
};


forrocket::Engine::Engine(const std::vector<double> time_vector, const std::vector<double> thrust_vacuum_vector, const std::vector<double> mdot_prop_vector, const double area_exit) {
    Reset();
    enable_gimbal = false;

    this->area_exit = area_exit;
    this->burn_duration = time_vector[time_vector.size()-1];
    thrust_vacuum_src = InterpolateParameter(time_vector, thrust_vacuum_vector, "zero");
    mdot_prop_src = InterpolateParameter(time_vector, mdot_prop_vector, "zero");

    getTotalImpulse(thrust_vacuum_vector, time_vector.back());
};


forrocket::Engine::Engine(const std::vector<double> time_vector, const std::vector<double> thrust_vacuum_vector, const std::vector<double> mdot_prop_vector, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis) {
    Reset();
    enable_gimbal = false;

    this->area_exit = area_exit;
    this->burn_duration = time_vector[time_vector.size()-1];
    this->mis_alignment_angle_y_axis = mis_alignment_angle_y_axis;
    this->mis_alignment_angle_z_axis = mis_alignment_angle_z_axis;

    thrust_vacuum_src = InterpolateParameter(time_vector, thrust_vacuum_vector, "zero");
    mdot_prop_src = InterpolateParameter(time_vector, mdot_prop_vector, "zero");

    getTotalImpulse(thrust_vacuum_vector, time_vector.back());
};


void forrocket::Engine::Update(const double t, const double pressure, const double mass_prop) {
    if (t > burn_duration || mass_prop <= 0.0) {
        Cutoff();
    }

    if (burning) {
        this->thrust = thrust_vacuum_src(t) - (pressure * area_exit);
        this->mdot_prop = mdot_prop_src(t);
    } else {
        thrust = 0.0;
        mdot_prop = 0.0;
    }

    gimbal_angle_y_axis = mis_alignment_angle_y_axis;
    gimbal_angle_z_axis = mis_alignment_angle_z_axis;
};

void forrocket::Engine::Ignittion() {
    burning = true;
};

void forrocket::Engine::Cutoff() {
    burning = false;
};




