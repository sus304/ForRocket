// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_stage_factory.hpp
// Creation Date   : 2020/02/06
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ROCKETSTAGEFACTORY_HPP_
#define ROCKETSTAGEFACTORY_HPP_

#include <string>

#include "solver/rocket_stage.hpp"


namespace forrocket {
    class RocketStageFactory {
        public:

            RocketStage Create(const int stage_number,
                                std::string rocket_config_json_file, 
                                std::string engine_config_json_file,
                                std::string sequence_of_event_json_file);
    };
}

#endif
