// ******************************************************
// Project Name    : ForRocket
// File Name       : DynamicsSolver.hpp
// Creation Date   : 2019/07/19
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef DynamicsSolver_hpp_
#define DynamicsSolver_hpp_

#include <iostream>
#include <vector>

#include "../lib/Eigen/Core"
#include "boost/numeric/odeint.hpp"

#include "Rocket.hpp"


namespace ForRocket
{
    namespace odeint = boost::numeric::odeint;

    void solve_trajectory(ForRocket::Rocket& rocket);

    class DynamicSolver{
        public:
            DynamicSolver();

            using state = std::array<double, 14>;

            void operator()(const state& x, state& dx, const double t);
        protected:
        void dynamics_6dof_aero_steardy();
        void dynamics_6dof_program_rate();
    };

}
#endif