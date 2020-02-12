// ******************************************************
// Project Name    : ForRocket
// File Name       : interpolate_parameter_factory.hpp
// Creation Date   : 2019/12/04
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef INTERPOLATEPARAMETERFACTORY_HPP_
#define INTERPOLATEPARAMETERFACTORY_HPP_

#include <string>

#include "rocket/parameter/interpolate_parameter.hpp"

namespace forrocket {
    class InterpolateParameterFactory {
        public:
            InterpolateParameter Create(const double value_const);
            InterpolateParameter Create(const std::string source_file_name);

    };
}

#endif
