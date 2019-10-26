// ******************************************************
// Project Name    : ForRocket
// File Name       : trajectory_solver.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "trajectory_solver.hpp"

#include "Eigen/Core"
#include "boost/numeric/odeint.hpp"

void forrocket::TrajectorySolver::solve(forrocket::DynamicsBase* p_dynamics, const double start_time, const double end_time, const double delta_time) {
    namespace odeint = boost::numeric::odeint;

    odeint::runge_kutta4<forrocket::DynamicsBase::state> stepper;

    forrocket::DynamicsBase::state x0 = {};
    odeint::integrate_const(stepper, *p_dynamics, x0, start_time, end_time, delta_time);

};

