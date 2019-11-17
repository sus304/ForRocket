// ******************************************************
// Project Name    : ForRocket
// File Name       : wind.hpp
// Creation Date   : 2019/10/26
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ENVIRONMENT_WIND_HPP_
#define ENVIRONMENT_WIND_HPP_

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    class EnvironmentWind {
        public:
            Eigen::Vector3d NED;

            void Update(const double altitude);
    };
}

#endif
