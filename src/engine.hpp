// ******************************************************
// Project Name    : ForRocket
// File Name       : engine.hpp
// Creation Date   : 2019/10/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ENGINE_HPP_
#define ENGINE_HPP_

#include <vector>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

#include "interpolate.hpp"

namespace forrocket {
    class Engine {
        public:
            double thrust;
            double mdot_prop;

            double area_exit;
            double burn_duration;

            double gimbal_angle_y_axis;  // Body coordinateの軸で回転角度の正負を定義
            double gimbal_angle_z_axis;

            double mis_alignment_angle_y_axis;
            double mis_alignment_angle_z_axis;

            Engine(const std::vector<double> time_vector, const std::vector<double> thrust_vector, const std::vector<double> mdot_p_vector);

            void Update(const double t, const double pressure_sea_level, const double pressure);
            void Ignition(const double pressure_sea_level, const double pressure);
            void Cutoff();

        private:
            std::vector<double> thrust_sea_level_vector;
            interpolate::Interp1d thrust_sea_level;
            std::vector<double> mdot_prop_vector;
    };
}

#endif
