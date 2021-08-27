// ******************************************************
// Project Name    : ForRocket
// File Name       : wind.hpp
// Creation Date   : 2019/10/26
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ENVIRONMENT_WIND_HPP_
#define ENVIRONMENT_WIND_HPP_

#include <string>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

#include "rocket/parameter/interpolate_parameter.hpp"


namespace forrocket {
    class EnvironmentWind {
        public:
            EnvironmentWind() {};
            EnvironmentWind(bool enable);  // enable=false => disableの時にしか呼ばない
            EnvironmentWind(std::string wind_file_path);

            Eigen::Vector3d getNED(const double altitude);

        private:
            InterpolateParameter wind_from_north;
            InterpolateParameter wind_from_east;

    };
}

#endif
