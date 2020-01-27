// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_base.hpp
// Creation Date   : 2019/10/20
//
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef DYNAMICSBASE_HPP_
#define DYNAMICSBASE_HPP_

#include <array>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

#include "rocket/rocket.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"

namespace forrocket {
    class DynamicsBase {
        public:
            DynamicsBase() {};

            using state = std::array<double, 14>;

            virtual void operator()(const state& x, state& dx, const double t) = 0;

        protected:
            Eigen::Vector3d AeroForce();

            Eigen::Vector3d GyroEffectMoment();
            Eigen::Vector3d ThrustMoment();
            Eigen::Vector3d AeroForceMoment();
            Eigen::Vector3d AeroDampingMoment();
            Eigen::Vector3d JetDampingMoment();
            
            Eigen::Vector3d QuaternionDiff();

        private:
            Rocket* p_rocket;
            SequenceClock* p_clock;
            EnvironmentWind* p_wind;


    };

}


#endif

