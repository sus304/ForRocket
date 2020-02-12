// ******************************************************
// Project Name    : ForRocket
// File Name       : interpolate_parameter.hpp
// Creation Date   : 2019/12/03
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef INTERPOLATEPARAMETER_HPP_
#define INTERPOLATEPARAMETER_HPP_

#include <vector>

#include "interpolate.hpp"

namespace forrocket {
    class InterpolateParameter {
        public:
            InterpolateParameter();
            InterpolateParameter(const double value);
            InterpolateParameter(const std::vector<double> x_axis_src, const std::vector<double> value_src, std::string fill_value);
            InterpolateParameter(const std::vector<double> x_axis_src, const std::vector<double> y_axis_src, const std::vector<double> value_src, std::string fill_value);
            InterpolateParameter(const InterpolateParameter& from);
            InterpolateParameter& operator=(const InterpolateParameter& from);

            double operator()(const double x_axis_value);
            double operator()(const double x_axis_value, const double y_axis_value);

        private:
            bool enable_1dlog;
            bool enable_2dlog;

            double value_const;
            interpolate::Interp1d polator_1d;
            interpolate::Interp2d polator_2d;

    };
}

#endif
