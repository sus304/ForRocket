// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_6dof_programrate.hpp
// Creation Date   : 2020/02/01
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef DYNAMICS6DOFPROGRAMRATE_HPP_
#define DYNAMICS6DOFPROGRAMRATE_HPP_

#include "dynamics/dynamics_base.hpp"
#include "rocket/rocket.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"

namespace forrocket {
    class Dynamics6dofProgramRate : public DynamicsBase {
        public:
            Dynamics6dofProgramRate(Rocket* rocket, SequenceClock* clock, EnvironmentWind* wind);

            void operator()(const state& x, state& dx, const double t);

        private:
            Rocket* p_rocket;
            SequenceClock* p_clock;
            EnvironmentWind* p_wind;
            
    };
}

#endif
