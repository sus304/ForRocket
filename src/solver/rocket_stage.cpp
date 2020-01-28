// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_stage.cpp
// Creation Date   : 2020/01/27
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket_stage.hpp"

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "boost/numeric/odeint.hpp"

#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"

#include "dynamics/dynamics_base.hpp"
#include "dynamics/dynamics_6dof_aero.hpp"

forrocket::RocketStage::RocketStage(Rocket rocket) {
    this->rocket = rocket;
};


void forrocket::RocketStage::FlightSequence(SequenceClock* master_clock, const EnvironmentWind* wind, DynamicsBase::state& x0) {
    namespace odeint = boost::numeric::odeint;
    odeint::runge_kutta4<forrocket::DynamicsBase::state> stepper;

    Dynamics6dofAero dynamics_6dof_aero(*rocket, master_clock, wind);
    // Dynamics6dofAero dynamics_6dof_aero();

    double start, end;
    double delta = 0.01;

    if (time_ignittion <= time_start) {
        rocket.IgnitionEngine(master_clock->UTC_date_init, master_clock->countup_time);
        start = time_start;
    } else {
        // Flight start から engine ignittion まで
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, time_start, time_ignittion, delta);
        start = time_ignittion;
        rocket.IgnitionEngine(master_clock->UTC_date_init, master_clock->countup_time);
    }
    
    if (enable_cutoff) {
        // engine cutoff まで 
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, time_cutoff, delta);
        start = time_cutoff;
        rocket.CutoffEngine();
    }

    if (enable_fairing_jettson) {
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, time_jettson_fairing, delta);
        start = time_jettson_fairing;
        rocket.JettsonFairing(mass_fairing);
    }

    if (enable_sepation) {
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, time_separation, delta);
        start = time_separation;
        rocket.SeparateUpperStage();

    if (enable_parachute_open) {
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, time_opan_parachute, delta);
        start = time_opan_parachute;
        // parachute syori?
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, time_end, delta);
    } else {
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, time_end, delta);
    }


};