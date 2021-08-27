// ******************************************************
// Project Name    : ForRocket
// File Name       : wgs84.hpp
// Creation Date   : 2019/10/27
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef WGS84_HPP_
#define WGS84_HPP_

#include <cmath>

namespace forrocket {
    class WGS84 {
        public:
            double a = 6378137.0;  // [m]  semi-major axis
            double inv_f = 298.257223563;  // inverse flattening factor
            double omega = 7292115e-11;  // [rad/s]
            double GM = 3.986004418e14;  // [m3/s2] geocentric gravitational constant

            double f = 1.0 / inv_f;
            double b = a * (1.0 - f);  // [m] semi-minor axis
            double e_square = 2.0 * f - std::pow(f, 2);
            double e = std::sqrt(e_square);  // 離心率

    };

}

#endif
