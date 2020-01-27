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

#include "rocket/parameter/interpolate_parameter.hpp"
#include "interpolate.hpp"

namespace forrocket {
    class Engine {
        public:
            bool burning;

            double thrust;
            double mdot_prop;

            double gimbal_angle_y_axis;  // Body coordinateの軸で回転角度の正負を定義
            double gimbal_angle_z_axis;

            Engine() {};

            // 固定推力、ミスアライメントなし
            Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit);
            // 固定推力、ミスアライメントあり
            Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis);

            // 時間推力、ミスアライメントなし
            Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit);
            // 時間推力、ミスアライメントあり
            Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis);
            
            void Update(const double t, const double pressure_sea_level, const double pressure);
            void Ignittion();
            void Cutoff();

        private:
            void Reset();

            double area_exit;
            double burn_duration;
            double mis_alignment_angle_y_axis;
            double mis_alignment_angle_z_axis;

            bool enable_gimbal;

            InterpolateParameter thrust_sea_level_src;
            InterpolateParameter mdot_prop_src;

    };
}

#endif
