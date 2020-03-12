// ******************************************************
// Project Name    : ForRocket
// File Name       : moment.hpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef MOMENT_HPP_
#define MOMENT_HPP_

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    class Moment {
        public:
            Eigen::Vector3d gyro;
            Eigen::Vector3d thrust;
            Eigen::Vector3d aero_force;
            Eigen::Vector3d aero_dumping;
            Eigen::Vector3d jet_dumping;

            Moment();
            Eigen::Vector3d Sum();
    };
}

#endif
