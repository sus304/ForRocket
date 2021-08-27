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

#include "Eigen/Core"
#include "Eigen/Dense"

#include "rocket/rocket.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"

namespace forrocket {
class DynamicsBase {
    public:
        // DynamicsBase() {};

        using state = std::array<double, 14>;

        virtual void operator()(const state& x, state& dx, const double t) = 0;

        Eigen::Vector3d AeroForce(Rocket* p_rocket);
        Eigen::Vector3d GyroEffectMoment(Rocket* p_rocket);
        Eigen::Vector3d ThrustMoment(Rocket* p_rocket);
        Eigen::Vector3d AeroForceMoment(Rocket* p_rocket);
        Eigen::Vector3d AeroDampingMoment(Rocket* p_rocket);
        Eigen::Vector3d JetDampingMoment(Rocket* p_rocket);
        
        Eigen::Matrix4d QuaternionDiff(Rocket* p_rocket);

    private:
        // Rocket* p_rocket;
        // SequenceClock* p_clock;
        // EnvironmentWind* p_wind;
};
}  // namespace forrocket


#endif

