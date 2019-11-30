// ******************************************************
// Project Name    : ForRocket
// File Name       : engine_factory.cpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "engine_factory.hpp"

#include <cmath>

const double pi = 3.14159265;

void forrocket::EngineFactory::Create(Engine& engine) {
    double diameter_throat = 15.0 / 1e3;
    double area_throat = 0.25 * std::pow(diameter_throat, 2) * pi;
    engine.area_exit = area_throat * 3.5;
    engine.Isp = 170.0;
    engine.burn_duration = 1.5;

    engine.mis_alignment_angle_y_axis = 0.0;
    engine.mis_alignment_angle_z_axis = 0.0;
};
