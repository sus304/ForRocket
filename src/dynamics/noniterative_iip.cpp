// ******************************************************
// Project Name    : ForRocket
// File Name       : noniterative_iip.cpp
// Creation Date   : 2020/02/24
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "noniterative_iip.hpp"

#include <cmath>

#include "degrad.hpp"
#include "environment/gravity.hpp"
#include "environment/wgs84.hpp"


std::pair<double, Eigen::Vector3d> forrocket::IIP(Eigen::Vector3d& pos_ECI, Eigen::Vector3d& vel_ECI) {
        // arg: Current ECI, Current ECI Vel
        // return IIP LLH [lat, lon, height]
        WGS84 wgs84;

        Eigen::Vector3d ir0;  // initial posint unit vector
        Eigen::Vector3d iv0;  // initial velocity unit vector
        double r0;  // initial position norm
        double v0;  // initial velocity norm
        double gamma0;  // inertia flight path angle

        Eigen::Vector3d ip;
        double rp;  // 地球中心とIIP間距離
        double rp_init;  // 収束計算用

        double vc;  // circular orbital velocity
        double lambda;  // squared speed ratio of current to circular orbital

        double c[3];
        double c_squr[3];

        double sin_phi;
        double cos_phi;
        double phi;
        
        double k[2];

        double lat_IIP_ECI;
        double lon_IIP_ECI;

        double lat_IIP_ECEF;
        double lon_IIP_ECEF;

        double tf1, tf1h, tf1l1, tf1l2;
        double tf2, tf21, tf2h, tf2l;
        double tf;

        Eigen::Vector3d IIP_LLH;

        // =================

        r0 = pos_ECI.norm();
        v0 = vel_ECI.norm();
        ir0 = pos_ECI.normalized();
        iv0 = vel_ECI.normalized();

        vc = std::sqrt(wgs84.GM / r0);
        gamma0 = std::asin(ir0.dot(iv0));  // [rad]
        lambda = std::pow(v0 / vc, 2);

        const double eps = 1e-6;
        const double relaxation = 0.1;
        rp_init = (wgs84.a + wgs84.b) * 0.5;
        // rp_init = wgs84.b;
        unsigned int counter = 0;
        while (std::abs(rp - rp_init) > eps) {
            rp = rp_init;

            c[0] = -std::tan(gamma0);
            c[1] = 1.0 - 1.0 / (lambda * std::pow(std::cos(gamma0), 2));
            c[2] = r0 / rp - 1.0 / (lambda * std::pow(std::cos(gamma0), 2));  // rp
            for (int i=0; i < 3; i++) {
                c_squr[i] = c[i] * c[i];
            }

            sin_phi = (c[0] * c[2]
                    + std::sqrt(c_squr[0] * c_squr[2] - (c_squr[0] + c_squr[1]) * (c_squr[2] - c_squr[1])))
                    / (c_squr[0] + c_squr[1]);
            cos_phi = std::sqrt(1.0 - sin_phi);
            phi = std::asin(sin_phi);  // [rad]

            k[0] = std::cos(gamma0 + phi) / std::cos(gamma0);
            k[1] = sin_phi / std::cos(gamma0);

            ip = k[0] * ir0 + k[1] * iv0;

            lat_IIP_ECI = std::asin(ip[2]);
            lon_IIP_ECI = std::atan2(ip[1], ip[0]);

            tf1h = std::tan(gamma0) * (1.0 - cos_phi) + (1.0 - lambda) * sin_phi;
            tf1l1 = (1.0 - cos_phi) / (lambda * std::pow(std::cos(gamma0), 2));
            tf1l2 = std::cos(gamma0 + phi) / std::cos(gamma0);
            tf1 = tf1h / ((2.0 - lambda) * (tf1l1 + tf1l2));

            tf21 = 2.0 * std::cos(gamma0) / (lambda * std::pow(2.0 / lambda - 1.0, 1.5));
            tf2h = std::sqrt(2.0 / lambda - 1.0);
            tf2l = std::cos(gamma0) * (1.0 / std::tan(phi * 0.5)) - std::sin(gamma0);
            tf2 = tf21 * std::atan2(tf2h , tf2l);

            tf = r0 / (v0 * std::cos(gamma0)) * (tf1 + tf2);

            lat_IIP_ECEF = lat_IIP_ECI;
            lon_IIP_ECEF = lon_IIP_ECI - wgs84.omega * tf;

            rp = wgs84.a * std::sqrt(1.0 - std::pow(wgs84.e * std::sin(lat_IIP_ECI), 2));
            
            counter++;
            double delta = rp - rp_init;
            if (rp > rp_init) {
                rp_init = rp_init + delta * relaxation;
            }
            else {
                rp_init = rp_init - delta * relaxation;
            }

        }
            c[2] = r0 / rp - 1.0 / (lambda * std::pow(std::cos(gamma0), 2));  // rp

        IIP_LLH[0] = rad2deg(lat_IIP_ECEF);
        IIP_LLH[1] = rad2deg(lon_IIP_ECEF);
        IIP_LLH[2] = 0.0;
        
        return std::make_pair(tf, IIP_LLH);
    };