// ******************************************************
// Project Name    : ForRocket
// File Name       : trajectory_solver.hpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef TRAJECTORYSOLVER_HPP_
#define TRAJECTORYSOLVER_HPP_

<<<<<<< HEAD

#include "solver/rocket_stage.hpp"
=======
#include "rocket/rocket.hpp"
>>>>>>> 54d51d7e23162f54ddfc33962da5f54d33c29dfc

namespace forrocket {
    class TrajectorySolver {
        public:
<<<<<<< HEAD
            TrajectorySolver() {};

            
            RocketStage stage_1st;
            RocketStage stage_2nd;
            RocketStage stage_3rd;
            RocketStage PBS;
            double mass_satellite;

            

            void Solve();

        private:
            void StageSolve(DynamicsBase::state x0);
=======
            void Solve();
>>>>>>> 54d51d7e23162f54ddfc33962da5f54d33c29dfc
    };
}

#endif
