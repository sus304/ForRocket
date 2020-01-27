// ******************************************************
// Project Name    : ForRocket
// File Name       : trajectory_solver.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "trajectory_solver.hpp"

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "boost/numeric/odeint.hpp"

#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"

#include "dynamics/dynamics_base.hpp"
#include "dynamics/dynamics_6dof_aero.hpp"

void forrocket::TrajectorySolver::Solve() {
    namespace odeint = boost::numeric::odeint;
    odeint::runge_kutta4<forrocket::DynamicsBase::state> stepper;

    // ECI pos, ECI vel, quat, angle vel, mass
    DynamicsBase::state x0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    odeint::integrate_const(stepper, p_dynamics, x0, start_time, end_time, delta_time);

    int stage_number = 0;
    double time_start = 0.0;
    double time_end = 0.0;
    double time_delta = 0.0;

    Dynamics6dofAero(stages[0], master_clock, )
    odeint::integrate_const(stepper, )


    // Lift off & B1 ig
    // Launcher Clear  => solver change

    // B1 Cut off
    // Faring jettson
    // B1 sepa  => solver change
    
    // B2 ig
    // B2 cut off
    // B2 sepa  => solver change
    
    // B3 ig
    // B3 cutoff
    // B3 sepa  => solver change
    
    // PBS ig
    // PBS cutoff
    // PBS sepa(Sat sepa)  => solve end

};

void forrocket::TrajectorySolver::StageSolve(RocketStage& stage, SequenceClock& master_clock, EnvironmentWind& wind, DynamicsBase::state& x0) {
    namespace odeint = boost::numeric::odeint;
    odeint::runge_kutta4<forrocket::DynamicsBase::state> stepper;

    Dynamics6dofAero dynamics_6dof_aero(&stage.rocket, &master_clock, &wind);

    double start, end;
    double delta = 0.01;

    if (stage.time_ignittion <= stage.time_start) {
        stage.rocket.IgnitionEngine(master_clock.UTC_date_init, master_clock.countup_time);
        start = stage.time_start;
    } else {
        // Flight start から engine ignittion まで
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, stage.time_start, stage.time_ignittion, delta);
        start = stage.time_ignittion;
        stage.rocket.IgnitionEngine(master_clock.UTC_date_init, master_clock.countup_time);
    }
    
    if (stage.enable_cutoff) {
        // engine cutoff まで 
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, stage.time_cutoff, delta);
        start = stage.time_cutoff;
        stage.rocket.CutoffEngine();
    }

    if (stage.enable_fairing_jettson) {
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, stage.time_jettson_fairing, delta);
        start = stage.time_jettson_fairing;
        stage.rocket.JettsonFairing(stage.mass_fairing);
    }

    if (stage.enable_sepation) {
        odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, stage.time_separation, delta);
        start = stage.time_separation;
        stage.rocket.SeparateUpperStage()




    } else if (stage.enable_parachute_open) {
        end = stage.time_opan_parachute;
    } else {
        end = 
    }
    odeint::integrate_const(stepper, dynamics_6dof_aero, x0, start, end, delta);



};

