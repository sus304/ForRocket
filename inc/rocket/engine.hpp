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

#include "Eigen/Core"

#include "rocket/parameter/interpolate_parameter.hpp"

namespace forrocket {
    class Engine {
        public:
            bool burning = false;
            double burn_duration = 0.0;

            double thrust = 0.0;
            double mdot_prop = 0.0;
            double total_impulse = 0.0;

            double gimbal_angle_y_axis = 0.0;  // Body coordinateの軸で回転角度の正負を定義
            double gimbal_angle_z_axis = 0.0;

            Engine() {};

            // 固定推力、ミスアライメントなし
            Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit);
            // 固定推力、ミスアライメントあり
            Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis);

            // 時間推力、ミスアライメントなし
            Engine(const std::vector<double> time_vector, const std::vector<double> thrust_vacuum_vector, const std::vector<double> mdot_prop_vector, const double area_exit);
            // 時間推力、ミスアライメントあり
            Engine(const std::vector<double> time_vector, const std::vector<double> thrust_vacuum_vector, const std::vector<double> mdot_prop_vector, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis);
            
            void Update(const double t, const double pressure, const double mass_prop);
            void Ignittion();
            void Cutoff();

        private:
            void Reset();
            void getTotalImpulse(const double thrust, const double burn_duration);
            void getTotalImpulse(const std::vector<double> thrust, const double burn_duration);

            double area_exit = 0.0;
            double mis_alignment_angle_y_axis = 0.0;
            double mis_alignment_angle_z_axis = 0.0;

            bool enable_gimbal = false;

            InterpolateParameter thrust_vacuum_src;
            InterpolateParameter mdot_prop_src;

    };
}

#endif
