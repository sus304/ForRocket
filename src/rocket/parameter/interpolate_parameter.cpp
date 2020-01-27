// ******************************************************
// Project Name    : ForRocket
// File Name       : interpolate_parameter.cpp
// Creation Date   : 2019/12/03
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "interpolate_parameter.hpp"


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




double forrocket::InterpolateParameter::operator()(const double x_axis_value) {
    if (enable_1dlog) return polator_1d(x_axis_value);
    else return value_const;
};


double forrocket::InterpolateParameter::operator()(const double x_axis_value, const double y_axis_value) {
    return 0.0;
};

