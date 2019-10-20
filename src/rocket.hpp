// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket.hpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ROCKET_HPP_
#define ROCKET_HPP_

#include <vector>

#include "position.hpp"
#include "velocity.hpp"
#include "mass.hpp"

namespace forrocket {
    class Rocket {
        public:
            Position position;
            Velocity velocity;
            Mass mass;

    };

}


#endif