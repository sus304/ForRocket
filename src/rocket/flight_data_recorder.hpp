// ******************************************************
// Project Name    : ForRocket
// File Name       : flight_data_recorder.hpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef FLIGHTDATARECORDER_HPP_
#define FLIGHTDATARECORDER_HPP_

#include <fstream>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

#include "environment/datetime.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"

#include "rocket/rocket.hpp"
#include "rocket/engine.hpp"
#include "rocket/parameter/position.hpp"
#include "rocket/parameter/velocity.hpp"
#include "rocket/parameter/acceleration.hpp"
#include "rocket/parameter/force.hpp"
#include "rocket/parameter/attitude.hpp"
#include "rocket/parameter/moment.hpp"
#include "rocket/parameter/mass.hpp"
#include "rocket/parameter/interpolate_parameter.hpp"

#include "dynamics/dynamics_base.hpp"

namespace forrocket {
    class FlightDataRecorder {
        public:
            FlightDataRecorder() {};
            FlightDataRecorder(Rocket* rocket);
            Rocket* p_rocket;

            std::vector<double> countup_time;
            std::vector<double> countup_burn_time;

            std::vector<double> thrust;
            std::vector<double> mdot_prop;
            std::vector<bool> burning;
            std::vector<double> gimbal_angle_y_axis;
            std::vector<double> gimbal_angle_z_axis;

            std::vector<double> mass_prop;
            std::vector<double> mass;
            std::vector<double> length_CG;
            std::vector<double> length_CP;
            std::vector<Eigen::Matrix3d> inertia_tensor;
            std::vector<double> CA;
            std::vector<double> CNa;
            std::vector<double> Cld;
            std::vector<double> Clp;
            std::vector<double> Cmq;

            std::vector<Position> position;
            std::vector<Velocity> velocity;
            std::vector<double> dynamic_pressure;
            std::vector<Acceleration> acceleration;
            std::vector<Force> force;
            std::vector<Attitude> attitude;
            std::vector<Eigen::Vector4d> quaternion_dot;
            std::vector<Eigen::Vector3d> angular_velocity;
            std::vector<Eigen::Vector3d> angular_acceleration;
            std::vector<double> angle_of_attack;
            std::vector<double> sideslip_angle;
            std::vector<Moment> moment;


            void ReserveCapacity(const int capacity);
            void operator()(const DynamicsBase::state& x, const double t);

            void DumpCsv(const std::string file_name, bool full_dump = true);
        
    };
};

#endif
