// ******************************************************
// Project Name    : ForRocket
// File Name       : environment_air.hpp
// Creation Date   : 2019/10/26
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ENVIRONMENTAIR_HPP_
#define ENVIRONMENTAIR_HPP_

namespace forrocket {
    class EnvironmentAir {
        public:
            EnvironmentAir() {};
            EnvironmentAir(const double altitude);

            double temprature;
            double pressure;
            double rho;
            double speed_of_sound;
    };
}

#endif
