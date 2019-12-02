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

            double gimbal_angle_y_axis;  // Body coordinateの軸で回転角度の正負を定義
            double gimbal_angle_z_axis;

            double mis_alignment_angle_y_axis;
            double mis_alignment_angle_z_axis;

            Engine() {};
            Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit);
            Engine(const double burn_duration, const double thrust_const, const double mdot_prop_const, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis);

            Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit);
            Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis);
            Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit,
                    const std::vector<double> gimbal_angle_y_axis_vector, const std::vector<double> gimbal_angle_z_axis_vector);
            Engine(const std::vector<double> time_vector, const std::vector<double> thrust_sea_level_vector, const std::vector<double> mdot_prop_vector, const double area_exit,
                    const std::vector<double> gimbal_angle_y_axis_vector, const std::vector<double> gimbal_angle_z_axis_vector,
                    const double mis_alignment_angle_y_axis, const double mis_alignment_angle_z_axis);

            void Update(const double t, const double pressure_sea_level, const double pressure);

        private:
            void Reset();

            double area_exit;

            bool enable_thrust_from_log;
            bool enable_gimbal;

            interpolate::Interp1d thrust_sea_level_polate;
            interpolate::Interp1d mdot_prop_polate;
            interpolate::Interp1d gimbal_angle_y_axis_polate;
            interpolate::Interp1d gimbal_angle_z_axis_polate;

            // For constant thrust
            double burn_duration;
            double thrust_const;
            double mdot_prop_const;
    };
}

#endif
