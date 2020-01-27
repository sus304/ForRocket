// ******************************************************
// Project Name    : ForRocket
// File Name       : engine.cpp
// Creation Date   : 2019/10/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "engine.hpp"

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

forrocket::Engine::Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit) {
    Reset();
    enable_gimbal = false;

    this->area_exit = area_exit;

    this->burn_duration = burn_duration;
    thrust_sea_level_src = InterpolateParameter(thrust_const);
    mdot_prop_src = InterpolateParameter(mdot_prop_const);
};


forrocket::Engine::Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis) {
    Reset();
    enable_gimbal = false;

    this->area_exit = area_exit;

    this->burn_duration = burn_duration;
    thrust_sea_level_src = InterpolateParameter(thrust_const);
    mdot_prop_src = InterpolateParameter(mdot_prop_const);
    this->mis_alignment_angle_y_axis = mis_alignment_angle_y_axis;
    this->mis_alignment_angle_z_axis = mis_alignment_angle_z_axis;
};


forrocket::Engine::Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit) {
    Reset();
    enable_gimbal = false;

    this->area_exit = area_exit;
    this->burn_duration = time_vector[time_vector.size()-1];
    thrust_sea_level_src = InterpolateParameter(time_vector, thrust_sea_level_vector, "zero");
    mdot_prop_src = InterpolateParameter(time_vector, mdot_prop_vector, "zero");

};


forrocket::Engine::Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis) {
    Reset();
    enable_gimbal = false;

    this->area_exit = area_exit;
    this->burn_duration = time_vector[time_vector.size()-1];
    this->mis_alignment_angle_y_axis = mis_alignment_angle_y_axis;
    this->mis_alignment_angle_z_axis = mis_alignment_angle_z_axis;

    thrust_sea_level_src = InterpolateParameter(time_vector, thrust_sea_level_vector, "zero");
    mdot_prop_src = InterpolateParameter(time_vector, mdot_prop_vector, "zero");
};


void forrocket::Engine::Update(const double t, const double pressure_sea_level, const double pressure) {
    if (t > burn_duration && burning) {
        Cutoff();
    }

    if (burning) {
        this->thrust = thrust_sea_level_src(t) + (pressure_sea_level - pressure) * area_exit;
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




