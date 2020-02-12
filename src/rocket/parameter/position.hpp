// ******************************************************
// Project Name    : ForRocket
// File Name       : position.hpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef POSITION_HPP_
#define POSITION_HPP_

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

#include "environment/datetime.hpp"
#include "environment/coordinate.hpp"

namespace forrocket {
class Position {
    public:
        Eigen::Vector3d ECI{0.0, 0.0, 0.0};
        Eigen::Vector3d ECEF{0.0, 0.0, 0.0};
        Eigen::Vector3d LLH{0.0, 0.0, 0.0};
        // double& altitude = LLH(2);

        Position();

        void Initialize(const DateTime datetime, const Eigen::Vector3d& LLH);
        void Update(Coordinate& coordinate, const Eigen::Vector3d& ECI);

};
}  // namespace forrocket


#endif