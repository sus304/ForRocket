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

#include "dynamics/dynamics_6dof_aero.hpp"
#include "dynamics/dynamics_6dof_programrate.hpp"
#include "dynamics/dynamics_3dof_onlauncher.hpp"
#include "dynamics/dynamics_3dof_parachute.hpp"

forrocket::RocketStage::RocketStage(int stage_number, Rocket rocket) {
    this->stage_number = stage_number;
    this->rocket = rocket;
    this->fdr = FlightDataRecorder(&this->rocket);
};


void forrocket::RocketStage::FlightSequence(SequenceClock* master_clock, EnvironmentWind* wind, DynamicsBase::state& x0) {
    namespace odeint = boost::numeric::odeint;
    odeint::runge_kutta_dopri5<forrocket::DynamicsBase::state> stepper;

    Dynamics6dofAero dynamics_6dof_aero(&rocket, master_clock, wind);
    Dynamics6dofProgramRate dynamics_6dof_programrate(&rocket, master_clock, wind);
    Dynamics3dofOnLauncher dynamics_3dof_onlauncher(&rocket, master_clock);
    Dynamics3dofParachute dynamics_3dof_parachute(&rocket, master_clock, wind);

    double start, end;
    double delta = 0.01;

    if (time_ignittion <= time_start) {
        // 計算開始と同時に点火する場合
        rocket.IgnitionEngine(master_clock->UTC_date_init, master_clock->countup_time);
        start = time_start;
    } else {
        // 点火が計算開始より後の場合
        // Flight start から engine ignittion まで
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, time_start, time_ignittion, delta, std::ref(fdr)); // 点火までの慣性飛行計算
        start = time_ignittion;
        rocket.IgnitionEngine(master_clock->UTC_date_init, master_clock->countup_time);
    }
    // ここまでにIgnitionEngineが実行されている

    if (enable_launcher) {
        Rocket rocket_on_launcher = rocket;
        SequenceClock clock_on_launcher = *master_clock;


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
        rocket.SeparateUpperStage(5.0);
    }

    if (enable_parachute_open) {
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, time_opan_parachute, delta);
        start = time_opan_parachute;
        // parachute syori?
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, time_end, delta);
    } else {
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, time_end, delta);
    }

    Stepper.initialize(System, State2, 0, 0.1);

};