// ******************************************************
// Project Name    : ForRocket
// File Name       : interpolate_parameter_factory.cpp
// Creation Date   : 2019/12/04
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "interpolate_parameter_factory.hpp"

#include <vector>

forrocket::InterpolateParameter forrocket::InterpolateParameterFactory::Create(const double value_const) {
    InterpolateParameter polate_parameter(value_const);
    return polate_parameter;
};


forrocket::InterpolateParameter forrocket::InterpolateParameterFactory::Create(const std::string source_file_name) {
    // file io
};

