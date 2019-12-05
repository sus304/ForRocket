// ******************************************************
// Project Name    : ForRocket
// File Name       : aerodynamics_parameter.cpp
// Creation Date   : 2019/12/03
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "aerodynamics_parameter.hpp"


forrocket::AerodynamicsParameter::AerodynamicsParameter(const double value) {
    enable_1dlog = false;
    enable_2dlog = false;

    value_const = value;
}


forrocket::AerodynamicsParameter::AerodynamicsParameter(const std::vector<double> mach_number_src, const std::vector<double> value_src) {
    enable_1dlog = true;
    enable_2dlog = false;

    polator_1d = interpolate::Interp1d(mach_number_src, value_src, "linear", "same");
};


forrocket::AerodynamicsParameter::AerodynamicsParameter(const std::vector<double> mach_number_src, const std::vector<double> AoA_src, const std::vector<double> value_src) {
    enable_1dlog = false;
    enable_2dlog = true;

    // polator_2d = interpolate::Interp2d(x_src, y_src, z_src, "linear", "same");
};




double forrocket::AerodynamicsParameter::operator()(const double mach_number) {
    if (enable_1dlog) return polator_1d(mach_number);
    else return value_const;
};


double forrocket::AerodynamicsParameter::operator()(const double mach_number, const double AoA) {
    return 0.0;
};

