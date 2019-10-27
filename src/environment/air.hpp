// ******************************************************
// Project Name    : ForRocket
// File Name       : air.hpp
// Creation Date   : 2019/10/26
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ENVIRONMENT_AIR_HPP_
#define ENVIRONMENT_AIR_HPP_

#include <array>

#include "environment/wgs84.hpp"
#include "environment/gravity.hpp"

namespace forrocket {
    class EnvironmentAir {
        public:
            EnvironmentAir() {};
            EnvironmentAir(const double altitude);

            double temprature;
            double pressure;
            double density;
            double speed_of_sound;
        
        private:
            WGS84 wgs84;

            // U.S. Standard Atomsphere 1976
            using Layer = std::array<double, 8>;
            Layer height_array = {0.0, 11.0e3, 20.0e3, 32.0e3, 47.0e3, 51.0e3, 71.0e3, 84.852e3};  // [m] geopotential height
            Layer temperature_gradient_array = {-6.5e-3, 0.0, 1.0e-3, 2.8e-3, 0, -2.8e-3, -2.0e-3, 0.0};  // [K/m] Temperature gradient
            Layer temperature_array = {288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946};  // [K]
            Layer pressure_array = {101325.0, 22632.0, 5474.9, 868.02, 110.91, 66.939, 3.9564, 0.3734};  // [Pa]

            double g0 = gravity(wgs84.a);
            double R = 287.1;
            double gamma = 1.4;
    };
}

#endif

