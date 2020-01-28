// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_stage.hpp
// Creation Date   : 2020/01/27
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ROCKETSTAGE_HPP_
#define ROCKETSTAGE_HPP_

#include "rocket/rocket.hpp"

namespace forrocket {
    class RocketStage {
        public:
            RocketStage() {};

            Rocket rocket;

            double time_start;
            double time_ignittion;

            bool on_lancher;
            
            bool enable_cutoff;
            double time_cutoff;

            bool enable_sepation;
            double time_separation;

            bool enable_fairing_jettson;
            double time_jettson_fairing;
            double mass_fairing;

            bool enable_parachute_open;
            double time_opan_parachute;

            double time_end;

    };
}

#endif
