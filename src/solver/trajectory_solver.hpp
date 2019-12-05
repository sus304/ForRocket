// ******************************************************
// Project Name    : ForRocket
// File Name       : trajectory_solver.hpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef TRAJECTORYSOLVER_HPP_
#define TRAJECTORYSOLVER_HPP_

#include "rocket/rocket.hpp"
#include "dynamics/dynamics_base.hpp"

namespace forrocket {
    class TrajectorySolver {
        public:
            void Solve(forrocket::DynamicsBase* p_dynamics, const double start_time, const double end_time, const double delta_time);
    };
}

#endif
