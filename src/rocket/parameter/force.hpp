// ******************************************************
// Project Name    : ForRocket
// File Name       : force.hpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef FORCE_HPP_
#define FORCE_HPP_

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    class Force {
        public:
            Eigen::Vector3d thrust;
            Eigen::Vector3d aero;
            Eigen::Vector3d gravity;

            Force();
            Eigen::Vector3d Sum();
    };
}

#endif
