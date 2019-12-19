// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_6dof.hpp
// Creation Date   : 2019/10/20
//
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef DYNAMICS6DOF_HPP_
#define DYNAMICS6DOF_HPP_

#include "dynamics/dynamics_base.hpp"
#include "rocket/rocket.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"

namespace forrocket {
    class Dynamics6dof : public DynamicsBase {
        public:
            Dynamics6dof() {};
            Dynamics6dof(Rocket* rocket, SequenceClock* clock, EnvironmentWind* wind);

            void operator()(const state& x, state& dx, const double t);

        private:
            Rocket* p_rocket;
            SequenceClock* p_clock;
            EnvironmentWind* p_wind;
            
    };
}

#endif
