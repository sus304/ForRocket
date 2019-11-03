// ******************************************************
// Project Name    : ForRocket
// File Name       : DynamicsSolver.cpp
// Creation Date   : 2019/07/19
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "DynamicsSolver.hpp"

void ForRocket::solve_trajectory(ForRocket::Rocket& rocket) {
    ForRocket::DynamicSolver::state state0;
    for (int i=0; i < state0.size(); i++) {
        state0[i] = 0.0;
    }
    ForRocket::DynamicSolver solver;
    odeint::runge_kutta4<DynamicSolver::state> stepper_RK4;

    double start_time = 0.0;
    double stop_time = 100.0;
    double step_time = 0.1;

    // odeint::integrate_const(stepper_RK4, solver, state0, start_time, stop_time, step_time);
}

ForRocket::DynamicSolver::DynamicSolver() {

}

void ForRocket::DynamicSolver::operator()(const state& x, state& dx, const double t) {

}

void ForRocket::DynamicSolver::dynamics_6dof_aero_steardy() {

}

void ForRocket::DynamicSolver::dynamics_6dof_program_rate() {

}
