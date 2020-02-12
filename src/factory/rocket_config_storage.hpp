// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_config_storage.hpp
// Creation Date   : 2020/02/06
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ROCKETCONFIGSTORAGE_HPP_
#define ROCKETCONFIGSTORAGE_HPP_

#include <string>

#include "json.hpp"

namespace forrocket {
    class RocketConfigStorage {
        public:
            RocketConfigStorage() {};

            void LoadConfigJsonFile(std::string filename);
    };
}

#endif
