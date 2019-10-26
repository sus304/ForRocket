// ******************************************************
// Project Name    : ForRocket
// File Name       : velocity.hpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef VELOCITY_HPP_
#define VELOCITY_HPP_

#include "Eigen/Core"

namespace forrocket {
    class Velocity {
        public:
            Eigen::Vector3d ECI;
            Eigen::Vector3d ECEF;
            Eigen::Vector3d NED;
            Eigen::Vector3d body;
            Eigen::Vector3d air_air;
            Eigen::Vector3d air_body;
            double mach_number;
    };

}


#endif