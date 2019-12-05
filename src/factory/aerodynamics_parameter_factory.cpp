// ******************************************************
// Project Name    : ForRocket
// File Name       : aerodynamics_parameter_factory.cpp
// Creation Date   : 2019/12/04
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "aerodynamics_parameter_factory.hpp"

#include <vector>

forrocket::AerodynamicsParameter forrocket::AerodynamicsParameterFactory::Create(const double value_const) {
    AerodynamicsParameter aero_parameter(value_const);
    return aero_parameter;
};


forrocket::AerodynamicsParameter forrocket::AerodynamicsParameterFactory::Create(const std::string source_file_name) {
    // file io
};

