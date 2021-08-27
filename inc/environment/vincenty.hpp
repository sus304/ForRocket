// ******************************************************
// Project Name    : ForRocket
// File Name       : vincenty.hpp
// Creation Date   : 2020/02/24
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef VINCENTY_HPP_
#define VINCENTY_HPP_

#include <utility>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    std::pair<double, double> vdownrange(Eigen::Vector3d& observer_LLH, Eigen::Vector3d& target_LLH);
}

#endif
