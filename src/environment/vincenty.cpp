// ******************************************************
// Project Name    : ForRocket
// File Name       : vincenty.cpp
// Creation Date   : 2020/02/24
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "vincenty.hpp"

#include <cmath>

#include "degrad.hpp"
#include "environment/wgs84.hpp"

std::pair<double, double> forrocket::vdownrange(Eigen::Vector3d& observer_LLH, Eigen::Vector3d& target_LLH) {
    // Input: [lat, lon, alt], [lat, lon, alt] ([deg, m])
    // Output: downrange [m], Azimuth start->end [deg]

    int itr_limit = 5000;

    if (observer_LLH(0) == target_LLH(0) && observer_LLH(1) == target_LLH(1)) {
        return std::make_pair(0.0, 0.0);
    }
    
    double lat1 = deg2rad(observer_LLH(0));
    double lon1 = deg2rad(observer_LLH(1));
    double lat2 = deg2rad(target_LLH(0));
    double lon2 = deg2rad(target_LLH(1));

    WGS84 wgs84;

    double U1 = std::atan((1.0 - wgs84.f) * std::tan(lat1));
    double U2 = std::atan((1.0 - wgs84.f) * std::tan(lat2));
    double diff_lon = lon2 - lon1;

    double sin_sigma = 0.0;
    double cos_sigma = 0.0;
    double sigma = 0.0;
    double sin_alpha = 0.0;
    double cos_alpha = 0.0;
    double cos_2sigma_m = 0.0;
    double coeff = 0.0;

    double lamda = diff_lon;
    for (int i=0; i < itr_limit; ++i) {
        sin_sigma = std::pow(std::cos(U2) * std::sin(lamda), 2) + std::pow(std::cos(U1) * std::sin(U2) - std::sin(U1) * std::cos(U2) * std::cos(lamda), 2);
        sin_sigma = std::sqrt(sin_sigma);
        cos_sigma = std::sin(U1) * std::sin(U2) + std::cos(U1) * std::cos(U2) * std::cos(lamda);
        sigma = std::atan2(sin_sigma, cos_sigma);

        sin_alpha = std::cos(U1) * std::cos(U2) * std::sin(lamda) / sin_sigma;
        cos_alpha = std::sqrt(1.0 - std::pow(sin_alpha, 2));

        cos_2sigma_m = cos_sigma - 2.0 * std::sin(U1) * std::sin(U2) / std::pow(cos_alpha, 2);

        coeff = wgs84.f / 16.0 * std::pow(cos_alpha, 2) * (4.0 + wgs84.f * (4.0 - 3.0 * std::pow(cos_alpha, 2)));
        double lamda_itr = lamda;
        lamda = diff_lon + (1.0 - coeff) * wgs84.f * sin_alpha * (sigma + coeff * sin_sigma * (cos_2sigma_m + coeff * cos_sigma * (-1.0 + 2.0 * cos_2sigma_m)));

        if (std::abs(lamda - lamda_itr) < 1e-12) {
            break;
        } // TODO: 収束しなかった時の処理追加
    }

    double u_squr = std::pow(cos_alpha, 2) * (std::pow(wgs84.a, 2) - std::pow(wgs84.b, 2)) / std::pow(wgs84.b, 2);
    double A = 1.0 + u_squr / 16384.0 * (4096.0 + u_squr * (-768.0 + u_squr * (320.0 - 175.0 * u_squr)));
    double B = u_squr / 1024.0 * (256.0 + u_squr * (-128.0 + u_squr * (74.0 - 47.0 * u_squr)));
    double delta_sigma = B * sin_sigma * (cos_2sigma_m + 0.25 * B *(cos_sigma * (-1.0 + 2.0 * std::pow(cos_2sigma_m, 2))
                        - (1.0 / 6.0) * B * cos_2sigma_m * (-3.0 + 4.0 * std::pow(sin_sigma, 2)) * (-3.0 + 4.0 * std::pow(cos_2sigma_m, 2))));
    
    double downrange = wgs84.b * A * (sigma - delta_sigma);
    double alpha1 = std::atan2(std::cos(U2) * std::sin(lamda), (std::cos(U1) * std::sin(U2) - std::sin(U1) * std::cos(U2) * std::cos(lamda)));  // observer to target azimuth
    double alpha2 = std::atan2(std::cos(U1) * std::sin(lamda), (-std::sin(U1) * std::cos(U2) + std::cos(U1) * std::sin(U2) * std::cos(lamda)));  // target to observer azimuth
    return std::make_pair(downrange, rad2deg(alpha1));
}

