// ******************************************************
// Project Name    : ForRocket
// File Name       : environment_wind.hpp
// Creation Date   : 2019/10/26
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ENVIRONMENTWIND_HPP_
#define ENVIRONMENTWIND_HPP_

#include "Eigen/Core"

namespace forrocket {
    class EnvironmentWind {
        public:
            Eigen::Vector3d NED;
    };
}

#endif
