// ******************************************************
// Project Name    : ForRocket
// File Name       : engine.cpp
// Creation Date   : 2019/10/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "engine.hpp"

void forrocket::Engine::Ignition(const double pressure_sea_level, const double pressure) {
    thrust = thrust_sea_level;
    mdot_prop = mdot_prop_source;
};


void forrocket::Engine::Update(const double t, const double pressure_sea_level, const double pressure) {
    thrust = thrust_sea_level + (pressure_sea_level - pressure) * area_exit;
    mdot_prop = mdot_prop_source;
};


void forrocket::Engine::Cutoff() {
    thrust = 0.0;
    mdot_prop = 0.0;
};

