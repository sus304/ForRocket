// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_factory.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket_factory.hpp"

#include <cmath>

#include "factory/engine_factory.hpp"
#include "rocket/parameter/interpolate_parameter.hpp"

#ifdef DEBUG
#include <iostream>
#endif

const double pi = 3.14159265;

forrocket::Rocket forrocket::RocketFactory::Create(std::string rocket_config_json_file, std::string engine_config_json_file) {
    Rocket rocket;

    EngineFactory engine_factory;
    rocket.engine = engine_factory.Create();

    #ifdef DEBUG
    std::cout << "Rocket Factory" << std::endl;
    #endif

    rocket.length_thrust = 0.3;
    rocket.diameter = 0.154;
    rocket.area = 0.25 * std::pow(rocket.diameter, 2) * pi;
    rocket.length = 2.0;
    rocket.mass.inert = 9.8;
    rocket.mass.propellant = 0.3;

    rocket.setLengthCG(InterpolateParameter(1.0));
    rocket.setLengthCP(InterpolateParameter(0.5));
    rocket.setCA(InterpolateParameter(0.4));
    rocket.setCNa(InterpolateParameter(9.0));
    rocket.setCld(InterpolateParameter(0.0));
    rocket.setClp(InterpolateParameter(0.01));
    rocket.setCmq(InterpolateParameter(0.1));
    rocket.setCnr(InterpolateParameter(0.1));
    rocket.cant_angle_fin = 0.0;
    rocket.setInertiaTensor(InterpolateParameter(0.3), InterpolateParameter(3.0), InterpolateParameter(3.0));

    rocket.setCdSParachute(1.5);

    return rocket;
};




