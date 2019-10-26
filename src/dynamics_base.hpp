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

#include "rocket.hpp"

namespace forrocket {
    class DynamicsBase {
        public:
            DynamicsBase() {};
            DynamicsBase(Rocket* rocket);

            using state = std::array<double, 14>;

            void operator()(const state& x, state& dx, const double t) {
                // ほんとはabstruct classにしたいがファンクタは純粋仮想関数にできないので
                // 使うなよという意味で強制終了を実装
                exit(1);
            };

        protected:
            Eigen::Vector3d ThrustBodyCoordinate();
            double getFinCantAxialForce();
            Eigen::Vector3d AeroForceBodyCoordinate();

            Eigen::Vector3d GyroEffectMoment();
            Eigen::Vector3d ThrustMoment();
            Eigen::Vector3d AeroForceMoment();
            Eigen::Vector3d AeroDampingMoment();
            Eigen::Vector3d JetDampingMoment();

        private:
            Rocket* p_rocket;



    };

}


#endif

