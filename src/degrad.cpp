// ******************************************************
// Project Name    : ForRocket
// File Name       : degrad.hpp
// Creation Date   : 2020/02/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "degrad.hpp"

double forrocket::deg2rad(const double deg) {
    return deg * pi / 180.0;
};

double forrocket::rad2deg(const double rad) {
    return rad * 180.0 / pi;
};

