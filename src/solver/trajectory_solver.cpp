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


void forrocket::TrajectorySolver::Solve() {
    namespace odeint = boost::numeric::odeint;
    odeint::runge_kutta4<forrocket::DynamicsBase::state> stepper;

    // ECI pos, ECI vel, quat, angle vel, mass
    DynamicsBase::state x0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    


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


