// ******************************************************
// Project Name    : ForRocket
// File Name       : velocity.hpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef VELOCITY_HPP_
#define VELOCITY_HPP_

#include "Eigen/Core"

#include "environment/datetime.hpp"
#include "environment/coordinate.hpp"

namespace forrocket {
    class Velocity {
        public:
            Eigen::Vector3d ECI{0.0, 0.0, 0.0};
            Eigen::Vector3d ECEF{0.0, 0.0, 0.0};
            Eigen::Vector3d NED{0.0, 0.0, 0.0};
            // Eigen::Vector3d body{0.0, 0.0, 0.0};
            Eigen::Vector3d air_body{0.0, 0.0, 0.0};
            double mach_number = 0.0;

            Velocity();

            void Initialize(const DateTime datetime, const Eigen::Vector3d& NED, const Eigen::Vector3d& pos_LLH, const Eigen::Vector3d& pos_ECI);
            void Update(Coordinate& coordinate, const Eigen::Vector3d& ECI, const Eigen::Vector3d& pos_ECI);
    };

}


#endif