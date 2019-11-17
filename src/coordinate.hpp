// ******************************************************
// Project Name    : ForRocket
// File Name       : coordinate.hpp
// Creation Date   : 2019/10/27
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef COORDINATE_HPP_
#define COORDINATE_HPP_

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    class Coordinate {
        public:

        Eigen::Matrix3d wind2body;
        wind2body << cos(alpha) * cos(beta), cos(alpha) * sin(beta), -sin(alpha),
            -sin(beta)             , cos(beta)             , 0.0,
            sin(alpha) * cos(beta), sin(alpha) * sin(beta), cos(alpha);
        
    };
}

#endif
