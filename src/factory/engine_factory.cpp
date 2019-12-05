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

forrocket::Engine forrocket::EngineFactory::Create() {
    double diameter_throat = 15.0 / 1e3;
    double area_throat = 0.25 * std::pow(diameter_throat, 2) * pi;
    double area_exit = area_throat * 3.5;

    double tb = 1.5;
    double thrust = 300.0;
    double Isp = 180.0;
    double mdot_p = thrust / (Isp * 9.80665);

    Engine engine(tb, thrust, mdot_p, area_exit);

    return engine;
};
