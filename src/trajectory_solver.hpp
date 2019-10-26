// ******************************************************
// Project Name    : ForRocket
// File Name       : trajectory_solver.hpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef TRAJECTORYSOLVER_HPP_
#define TRAJECTORYSOLVER_HPP_

#include "rocket.hpp"
#include "dynamics_base.hpp"

namespace forrocket {
    class TrajectorySolver {
        public:
            void solve(forrocket::DynamicsBase* p_dynamics, const double start_time, const double end_time, const double delta_time);
    };
}

#endif
