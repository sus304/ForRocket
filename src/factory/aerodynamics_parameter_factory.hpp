// ******************************************************
// Project Name    : ForRocket
// File Name       : aerodynamics_parameter_factory.hpp
// Creation Date   : 2019/12/04
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef AERODYNAMICSPARAMETERFACTORY_HPP_
#define AERODYNAMICSPARAMETERFACTORY_HPP_

#include <string>

#include "rocket/parameter/aerodynamics_parameter.hpp"

namespace forrocket {
    class AerodynamicsParameterFactory {
        public:
            AerodynamicsParameter Create(const double value_const);
            AerodynamicsParameter Create(const std::string source_file_name);

    };
}

#endif
