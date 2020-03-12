// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_3dof_parachute.hpp
// Creation Date   : 2020/02/01
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef DYNAMICS3DOFPARACHUTE_HPP_
#define DYNAMICS3DOFPARACHUTE_HPP_

#include "dynamics/dynamics_base.hpp"
#include "rocket/rocket.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"

namespace forrocket {
    class Dynamics3dofParachute : public DynamicsBase {
        public:
            Dynamics3dofParachute(Rocket* rocket, SequenceClock* clock, EnvironmentWind* wind);

            void operator()(const state& x, state& dx, const double t);

        private:
            Rocket* p_rocket;
            SequenceClock* p_clock;
            EnvironmentWind* p_wind;
            
    };
}

#endif
