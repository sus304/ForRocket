// ******************************************************
// Project Name    : ForRocket
// File Name       : attitude.hpp
// Creation Date   : 2019/12/04
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ATTITUDE_HPP_
#define ATTITUDE_HPP_

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    class Attitude {
        public:
            Eigen::Vector3d euler_angle;
            Eigen::Vector4d quaternion;

            Attitude();
            
        private:
    };
}

#endif
