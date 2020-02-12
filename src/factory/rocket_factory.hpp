// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_factory.hpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ROCKETFACTORY_HPP_
#define ROCKETFACTORY_HPP_

#include <string>

#include "rocket/rocket.hpp"

namespace forrocket {
    class RocketFactory {
        public:
            Rocket Create(std::string rocket_config_json_file, std::string engine_config_json_file);
    };
}

#endif
