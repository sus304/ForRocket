// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket.hpp
// Creation Date   : 2019/10/20
//
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ROCKET_HPP_
#define ROCKET_HPP_

#include <vector>

#include "Eigen/Core"

#include "engine.hpp"
#include "position.hpp"
#include "velocity.hpp"
#include "acceleration.hpp"
#include "mass.hpp"

#include "environment/air.hpp"

namespace forrocket {
    class Rocket {
        public:
            Engine engine;
            double length_thrust;  // from end

            double diameter;
            double area;
            double length;
            Mass mass;
            double length_CG;  // from end
            double length_CP;  // from end
            Eigen::Matrix3d inertia_tensor;
            // Eigen::Vector3d inertia_moment;

            double CA;
            double CNa;
            double Clp;
            double Cmq;
            double Cnr;

            double cant_angle_fin;  // rollモーメント正となるフィンカント角を正で定義
            double CNa_fin;
            double radius_CP_fin;

            Position position;
            Velocity velocity;
            double dynamic_pressure;
            Acceleration acceleration;

            Eigen::Vector3d force_thrust;
            Eigen::Vector3d force_aero;

            Eigen::Vector4d quaternion;
            Eigen::Vector4d quaternion_dot;
            Eigen::Vector3d omega;
            Eigen::Vector3d omega_dot;
            double angle_of_attack;
            double sideslip_angle;

            Eigen::Vector3d moment_aero_force;
            Eigen::Vector3d moment_aero_dumping;
            Eigen::Vector3d moment_jet_dumping;

            void UpdatePosition();
            void UpdateVelocity();
            void UpdateLengthCG(const double t);
            void UpdateLengthCP(const double mach);
            void UpdateEngine(const double t, const double air_pressure);
            void UpdateAerodynamicsCoefficient(const double mach);
            void UpdateAoA(Eigen::Vector3d velocity_air_body);
            void UpdateAttitudeFromProgramRate(const double t);


            // void SeparateUpperStage();

        private:
            // bool has_upper_stage;

    };

}


#endif