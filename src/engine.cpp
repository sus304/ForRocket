// ******************************************************
// Project Name    : ForRocket
// File Name       : engine.cpp
// Creation Date   : 2019/10/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "engine.hpp"

void forrocket::Engine::Reset() {
    thrust = 0.0;
    mdot_prop = 0.0;
    gimbal_angle_y_axis = 0.0;
    gimbal_angle_z_axis = 0.0;
    mis_alignment_angle_y_axis = 0.0;
    mis_alignment_angle_z_axis = 0.0;

    enable_thrust_from_log = false;
    enable_gimbal = false;

    area_exit = 0.0;
    burn_duration = 0.0;
    thrust_const = 0.0;
    mdot_prop_const = 0.0;
};

forrocket::Engine::Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit) {
    Reset();
    enable_thrust_from_log = false;
    enable_gimbal = false;

    this->area_exit = area_exit;

    this->burn_duration = burn_duration;
    this->thrust_const = thrust_const;
    this->mdot_prop_const = mdot_prop_const;
};


forrocket::Engine::Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis) {
    Reset();
    enable_thrust_from_log = false;
    enable_gimbal = false;

    this->area_exit = area_exit;

    this->burn_duration = burn_duration;
    this->thrust_const = thrust_const;
    this->mdot_prop_const = mdot_prop_const;
    this->mis_alignment_angle_y_axis = mis_alignment_angle_y_axis;
    this->mis_alignment_angle_z_axis = mis_alignment_angle_z_axis;
};


forrocket::Engine::Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit) {
    Reset();
    enable_thrust_from_log = true;
    enable_gimbal = false;

    this->area_exit = area_exit;

    thrust_sea_level_polate = interpolate::Interp1d(time_vector, thrust_sea_level_vector, "linear", "zero");
    mdot_prop_polate = interpolate::Interp1d(time_vector, mdot_prop_vector, "linear", "zero");
};


forrocket::Engine::Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis) {
    Reset();
    enable_thrust_from_log = true;
    enable_gimbal = false;

    this->area_exit = area_exit;
    this->mis_alignment_angle_y_axis = mis_alignment_angle_y_axis;
    this->mis_alignment_angle_z_axis = mis_alignment_angle_z_axis;

    thrust_sea_level_polate = interpolate::Interp1d(time_vector, thrust_sea_level_vector, "linear", "zero");
    mdot_prop_polate = interpolate::Interp1d(time_vector, mdot_prop_vector, "linear", "zero");
};


forrocket::Engine::Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit,
                    const std::vector<double> gimbal_angle_y_axis_vector, const std::vector<double> gimbal_angle_z_axis_vector) {
    Reset();
    enable_thrust_from_log = true;
    enable_gimbal = true;

    this->area_exit = area_exit;

    thrust_sea_level_polate = interpolate::Interp1d(time_vector, thrust_sea_level_vector, "linear", "zero");
    mdot_prop_polate = interpolate::Interp1d(time_vector, mdot_prop_vector, "linear", "zero");
    gimbal_angle_y_axis_polate = interpolate::Interp1d(time_vector, gimbal_angle_y_axis_vector, "linear", "same");
    gimbal_angle_z_axis_polate = interpolate::Interp1d(time_vector, gimbal_angle_z_axis_vector, "linear", "same");

};


forrocket::Engine::Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit,
                    const std::vector<double> gimbal_angle_y_axis_vector, const std::vector<double> gimbal_angle_z_axis_vector,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis) {
    Reset();
    enable_thrust_from_log = true;
    enable_gimbal = true;

    this->area_exit = area_exit;
    this->mis_alignment_angle_y_axis = mis_alignment_angle_y_axis;
    this->mis_alignment_angle_z_axis = mis_alignment_angle_z_axis;

    thrust_sea_level_polate = interpolate::Interp1d(time_vector, thrust_sea_level_vector, "linear", "zero");
    mdot_prop_polate = interpolate::Interp1d(time_vector, mdot_prop_vector, "linear", "zero");
    gimbal_angle_y_axis_polate = interpolate::Interp1d(time_vector, gimbal_angle_y_axis_vector, "linear", "same");
    gimbal_angle_z_axis_polate = interpolate::Interp1d(time_vector, gimbal_angle_z_axis_vector, "linear", "same");
};



void forrocket::Engine::Update(const double t, const double pressure_sea_level, const double pressure) {
    if (enable_thrust_from_log) {
        thrust = thrust_sea_level_polate(t) + (pressure_sea_level - pressure) * area_exit;
        mdot_prop = mdot_prop_polate(t);
    } else {
        if (t < burn_duration) {
            thrust = thrust_const + (pressure_sea_level - pressure) * area_exit;
            mdot_prop = mdot_prop_const;
        } else {
            thrust = 0.0;
            mdot_prop = 0.0;
        }
    }

    if (enable_gimbal) {
        gimbal_angle_y_axis = gimbal_angle_y_axis_polate(t) + mis_alignment_angle_y_axis;
        gimbal_angle_y_axis = gimbal_angle_z_axis_polate(t) + mis_alignment_angle_z_axis;
    } else {
        gimbal_angle_y_axis = mis_alignment_angle_y_axis;
        gimbal_angle_z_axis = mis_alignment_angle_z_axis;
    }
};



