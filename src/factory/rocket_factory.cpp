// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_factory.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket_factory.hpp"

#include <cmath>

#include "factory/engine_factory.hpp"
#include "factory/aerodynamics_parameter_factory.hpp"

const double pi = 3.14159265;

forrocket::Rocket forrocket::RocketFactory::Create(const DateTime datetime) {
    EngineFactory engine_factory;
    AerodynamicsParameterFactory aero_factory;

    Rocket rocket;

    rocket.length_thrust = 0.3;
    rocket.diameter = 0.154;
    rocket.area = 0.25 * std::pow(rocket.diameter, 2) * pi;
    rocket.length = 2.0;
    rocket.mass.inert = 9.8;
    rocket.mass.propellant = 0.3;

    rocket.setLengthCG(1.0);

    rocket.setLengthCP(aero_factory.Create(0.5));
    rocket.inertia_tensor << 0.3, 0, 0, 0, 3.0, 0, 0, 0, 3.0;
    rocket.setCA(aero_factory.Create(0.5));
    rocket.setCNa(aero_factory.Create(9.0));
    rocket.setClp(aero_factory.Create(0.02));
    rocket.setCmq(aero_factory.Create(0.4));
    rocket.setCnr(aero_factory.Create(0.4));
    rocket.cant_angle_fin = 0.0;
    rocket.CNa_fin = 8.0;
    rocket.radius_CP_fin = 0.17;

    rocket.engine = engine_factory.Create();

    Eigen::Vector3d llh;
    llh << 35.2, 135.3, 0.0;
    rocket.position.Initialize(datetime, llh);
    Eigen::Vector3d ned;
    ned << 0.0, 0.0, 0.0;
    rocket.velocity.Initialize(datetime, ned, llh, rocket.position.ECI);

    double elv_init = 85.0;
    double azi_init = 275.0;
    double roll_init = 0.0;

    return rocket;
};




