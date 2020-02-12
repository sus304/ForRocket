// ******************************************************
// Project Name    : ForRocket
// File Name       : interpolate_parameter.cpp
// Creation Date   : 2019/12/03
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "interpolate_parameter.hpp"

#ifdef DEBUG
#include <iostream>
#endif

forrocket::InterpolateParameter::InterpolateParameter() {
    enable_1dlog = false;
    enable_2dlog = false;

    value_const = 0.0;
};


forrocket::InterpolateParameter::InterpolateParameter(const double value) {
    enable_1dlog = false;
    enable_2dlog = false;

    value_const = value;
}


forrocket::InterpolateParameter::InterpolateParameter(const std::vector<double> x_axis_src, const std::vector<double> value_src, std::string fill_value) {
    enable_1dlog = true;
    enable_2dlog = false;

    polator_1d = interpolate::Interp1d(x_axis_src, value_src, "linear", fill_value);
};


forrocket::InterpolateParameter::InterpolateParameter(const std::vector<double> x_axis_src, const std::vector<double> y_axis_src, const std::vector<double> value_src, std::string fill_value) {
    enable_1dlog = false;
    enable_2dlog = true;

    // polator_2d = interpolate::Interp2d(x_src, y_src, z_src, "linear", fill_value);
};

forrocket::InterpolateParameter::InterpolateParameter(const InterpolateParameter& from) {
    enable_1dlog = from.enable_1dlog;
    enable_2dlog = from.enable_2dlog;

    if (from.enable_1dlog) polator_1d = from.polator_1d;
    else if (from.enable_2dlog) polator_2d = from.polator_2d;
    else {
        #ifdef DEBUG
        std::cout << "IP value copy" << std::endl;
        #endif
        value_const = from.value_const;
    }
};

forrocket::InterpolateParameter& forrocket::InterpolateParameter::operator=(const InterpolateParameter& from) {
    if (this != &from) {
        enable_1dlog = from.enable_1dlog;
        enable_2dlog = from.enable_2dlog;

        if (from.enable_1dlog) polator_1d = from.polator_1d;
        else if (from.enable_2dlog) polator_2d = from.polator_2d;
        else {
            #ifdef DEBUG
            std::cout << "IP value eq" << std::endl;
            std::cout << from.value_const << std::endl;
            #endif
            value_const = from.value_const;
        }
    }
    return *this;
};


double forrocket::InterpolateParameter::operator()(const double x_axis_value) {
    if (enable_1dlog) return polator_1d(x_axis_value);
    else return value_const;
};


double forrocket::InterpolateParameter::operator()(const double x_axis_value, const double y_axis_value) {
    return 0.0;
};

