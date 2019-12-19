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

#include "dynamics/dynamics_base.hpp"

void forrocket::TrajectorySolver::Solve() {
    namespace odeint = boost::numeric::odeint;
    odeint::runge_kutta4<forrocket::DynamicsBase::state> stepper;



    forrocket::DynamicsBase::state x0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    odeint::integrate_const(stepper, p_dynamics, x0, start_time, end_time, delta_time);

};

