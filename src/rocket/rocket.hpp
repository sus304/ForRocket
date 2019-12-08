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

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

#include "rocket/engine.hpp"
#include "rocket/parameter/position.hpp"
#include "rocket/parameter/velocity.hpp"
#include "rocket/parameter/acceleration.hpp"
#include "rocket/parameter/force.hpp"
#include "rocket/parameter/attitude.hpp"
#include "rocket/parameter/moment.hpp"
#include "rocket/parameter/mass.hpp"
#include "rocket/parameter/aerodynamics_parameter.hpp"
#include "interpolate.hpp"

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
            Force force;

            Attitude attitude;
            Eigen::Vector4d quaternion_dot;
            Eigen::Vector3d angular_velocity;
            Eigen::Vector3d angular_acceleration;
            double angle_of_attack;
            double sideslip_angle;
            Moment moment;

            Rocket();

            void UpdateLengthCG(const double t);
            // void UpdateInertiaTensor(const double t);
            void UpdateAerodynamicsParameter(const double mach_number);
            void UpdateAerodynamicsParameter(const double mach_number, const double AoA);
            void UpdateAttitudeFromProgramRate(const double t);

            void setLengthCG(const double length_CG);
            void setLengthCG(const std::vector<double> time_vector, const std::vector<double> CG_vector);

            void setLengthCP(const AerodynamicsParameter length_CP);
            void setCA(const AerodynamicsParameter CA);
            void setCNa(const AerodynamicsParameter CNa);
            void setClp(const AerodynamicsParameter Clp);
            void setCmq(const AerodynamicsParameter Cmq);
            void setCnr(const AerodynamicsParameter Cnr);

        private:
            bool enable_length_CG_from_log;
            bool enable_program_rate;

            interpolate::Interp1d length_CG_polate;
            interpolate::Interp1d program_rate_polate;

            AerodynamicsParameter length_CP_src;
            AerodynamicsParameter CA_src;
            AerodynamicsParameter CNa_src;
            AerodynamicsParameter Clp_src;
            AerodynamicsParameter Cmq_src;
            AerodynamicsParameter Cnr_src;

    };

}


#endif