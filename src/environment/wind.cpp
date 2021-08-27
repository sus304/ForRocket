// ******************************************************
// Project Name    : ForRocket
// File Name       : wind.cpp
// Creation Date   : 2019/10/26
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "environment/wind.hpp"

#include "fileio.hpp"

forrocket::EnvironmentWind::EnvironmentWind(bool enable) {
    // enable=false => disableの時にしか呼ばない
    // TODO: 分かりづらいのでなんか良い方法考える
    wind_from_east = InterpolateParameter(0.0);
    wind_from_north = InterpolateParameter(0.0);
};


forrocket::EnvironmentWind::EnvironmentWind(std::string wind_file_path) {
    auto wind_log = LoadCsvLog(wind_file_path);

    wind_from_east = InterpolateParameter(wind_log[0], wind_log[1], "zero");
    wind_from_north = InterpolateParameter(wind_log[0], wind_log[2], "zero");
};


Eigen::Vector3d forrocket::EnvironmentWind::getNED(const double altitude) {
    Eigen::Vector3d NED;

    NED << wind_from_north(altitude), wind_from_east(altitude), 0.0;

    return NED;
};
