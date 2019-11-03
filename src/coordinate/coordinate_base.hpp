// ******************************************************
// Project Name    : ForRocket
// File Name       : coordinate_base.hpp
// Creation Date   : 2019/10/28
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef COORDINATEBASE_HPP_
#define COORDINATEBASE_HPP_

#include "Eigen/Core"

namespace forrocket {
    class CoordinateBase {
        public:
            virtual Eigen::Vector3d toECEF() = 0;

    };
}

#endif
