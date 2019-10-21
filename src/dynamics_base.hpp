// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_base.hpp
// Creation Date   : 2019/10/20
//
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef DYNAMICSBASE_HPP_
#define DYNAMICSBASE_HPP_

#include <array>

namespace forrocket {
    class DynamicsBase {
        public:
            using state = std::array<double, 14>;
    };

}


#endif

