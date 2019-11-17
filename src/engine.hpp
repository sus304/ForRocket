// ******************************************************
// Project Name    : ForRocket
// File Name       : engine.hpp
// Creation Date   : 2019/10/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ENGINE_HPP_
#define ENGINE_HPP_

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

namespace forrocket {
    class Engine {
        public:
            double thrust;
            double area_exit;
            double mdot_prop;
            double Isp;
            double burn_duration;

            double gimbal_angle_y_axis;  // Body coordinateの軸で回転角度の正負を定義
            double gimbal_angle_z_axis;

            double mis_alignment_angle_y_axis;
            double mis_alignment_angle_z_axis;

            void Ignition(const double pressure_sea_level, const double pressure);
            void Update(const double t, const double pressure_sea_level, const double pressure);
            void Cutoff();

        private:
            double thrust_sea_level;
            double mdot_prop_source;
    };
}

#endif
