// ******************************************************
// Project Name    : ForRocket
// File Name       : noniterative_iip.hpp
// Creation Date   : 2020/02/24
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef NONITERATIVEIIP_HPP_
#define NONITERATIVEIIP_HPP_

#include <utility>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    std::pair<double, Eigen::Vector3d> IIP(Eigen::Vector3d& pos_ECI, Eigen::Vector3d& vel_ECI);
}

#endif
