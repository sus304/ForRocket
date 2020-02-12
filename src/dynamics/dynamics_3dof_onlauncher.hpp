// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_3dof_onlauncher.hpp
// Creation Date   : 2020/02/01
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef DYNAMICS3DOFONLAUNCHER_HPP_
#define DYNAMICS3DOFONLAUNCHER_HPP_

#include "dynamics/dynamics_base.hpp"
#include "rocket/rocket.hpp"
#include "environment/sequence_clock.hpp"

namespace forrocket {
    class Dynamics3dofOnLauncher : public DynamicsBase {
        public:
            Dynamics3dofOnLauncher(Rocket* rocket, SequenceClock* clock);

            void operator()(const state& x, state& dx, const double t);

            Eigen::Vector3d AeroForce();

        private:
            Rocket* p_rocket;
            SequenceClock* p_clock;
            
    };
}

#endif
