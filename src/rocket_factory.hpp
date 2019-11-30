// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_factory.hpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ROCKETFACTORY_HPP_
#define ROCKETFACTORY_HPP_

#include "rocket.hpp"
#include "datetime.hpp"

namespace forrocket {
    class RocketFactory {
        Rocket* Create(const DateTime datetime);
    }
}

#endif
