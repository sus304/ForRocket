// ******************************************************
// Project Name    : ForRocket
// File Name       : aerodynamics_parameter.hpp
// Creation Date   : 2019/12/03
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef AERODYNAMICSPARAMETER_HPP_
#define AERODYNAMICSPARAMETER_HPP_

#include <vector>

#include "interpolate.hpp"

namespace forrocket {
    class AerodynamicsParameter {
        public:
            AerodynamicsParameter(const double value);
            AerodynamicsParameter(const std::vector<double> mach_number_src, const std::vector<double> value_src);
            AerodynamicsParameter(const std::vector<double> mach_number_src, const std::vector<double> AoA_src, const std::vector<double> value_src);

            double operator()(const double mach_number);
            double operator()(const double mach_number, const double AoA);

        private:
            bool enable_1dlog;
            bool enable_2dlog;

            double value_const;
            interpolate::Interp1d polator_1d;
            interpolate::Interp2d polator_2d;

    };
}

#endif
