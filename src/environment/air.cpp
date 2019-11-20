// ******************************************************
// Project Name    : ForRocket
// File Name       : air.cpp
// Creation Date   : 2019/10/26
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "environment/air.hpp"

#include <cmath>

void forrocket::EnvironmentAir::Update(const double geometric_altitude) {
    double geopotential_height = geometric_altitude * wgs84.a / (wgs84.a + geometric_altitude);

    int index_layer = 0;  // default layer
    for (int i=0; i < height_array.size(); ++i) {
        if (geopotential_height < height_array[i]) {
            index_layer = i - 1;
            if (index_layer < 0) {
                index_layer = 0;
            }
            break;
        }
        else if (geopotential_height >= height_array.back()) {
            index_layer = 7;
            break;
        }
    }

    temprature = temperature_array[index_layer] + temperature_gradient_array[index_layer] * (geopotential_height - height_array[index_layer]);  // [K]
    if (temperature_gradient_array[index_layer] == 0.0) {
        pressure = pressure_array[index_layer] * std::exp(g0 / R * (height_array[index_layer] - geopotential_height) / temperature_array[index_layer]);  // [Pa]
    }
    else {
        pressure = pressure_array[index_layer] * std::pow(temperature_array[index_layer] / temprature, g0 / R / temperature_gradient_array[index_layer]);
    }
    density = pressure / (R * temprature);  // [kg/m3]
    speed_of_sound = std::sqrt(gamma * R * temprature);
};
