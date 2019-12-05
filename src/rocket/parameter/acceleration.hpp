// ******************************************************
// Project Name    : ForRocket
// File Name       : acceleration.hpp
// Creation Date   : 2019/10/26
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ACCELERATION_HPP_
#define ACCELERATION_HPP_

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    class Acceleration {
        public:
            Eigen::Vector3d ECI;
            Eigen::Vector3d body;

            Acceleration();
    };
}

#endif
