// ******************************************************
// Project Name    : ForRocket
// File Name       : attitude.hpp
// Creation Date   : 2019/12/04
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ATTITUDE_HPP_
#define ATTITUDE_HPP_

#include "Eigen/Core"

#include "environment/coordinate.hpp"

namespace forrocket {
    class Attitude {
        public:
            Eigen::Vector3d euler_angle;  // radian
            Eigen::Vector4d quaternion;

            Attitude();

            void Initialize(const Eigen::Vector3d& euler_angle);
            void Update(const Eigen::Vector4d& quaternion, Coordinate& coordinate);
            
        private:

    };
}

#endif
