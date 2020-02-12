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

#include "environment/datetime.hpp"
#include "environment/sequence_clock.hpp"

#include "rocket/engine.hpp"
#include "rocket/parameter/position.hpp"
#include "rocket/parameter/velocity.hpp"
#include "rocket/parameter/acceleration.hpp"
#include "rocket/parameter/force.hpp"
#include "rocket/parameter/attitude.hpp"
#include "rocket/parameter/moment.hpp"
#include "rocket/parameter/mass.hpp"
#include "rocket/parameter/interpolate_parameter.hpp"
#include "interpolate.hpp"

namespace forrocket {
    class Rocket {
        public:
            Rocket();

            SequenceClock burn_clock;
            Engine engine;
            double length_thrust = 0.0;  // from end

            double diameter = 0.0;
            double area = 0.0;
            double length = 0.0;
            double length_CG = 0.0;
            double length_CP = 0.0;
            Mass mass;
            Eigen::Matrix3d inertia_tensor;
            
            double CA = 0.0;
            double CNa = 0.0;
            double Cld = 0.0;
            double Clp = 0.0;
            double Cmq = 0.0;
            double Cnr = 0.0;
            double cant_angle_fin = 0.0;  // rollモーメント正となるフィンカント角を正で定義

            Position position;
            Velocity velocity;
            double dynamic_pressure = 0.0;
            Acceleration acceleration;
            Force force;

            Attitude attitude;
            Eigen::Vector4d quaternion_dot;
            Eigen::Vector3d angular_velocity;
            Eigen::Vector3d angular_acceleration;
            double angle_of_attack = 0.0;
            double sideslip_angle = 0.0;
            Moment moment;

            double CdS_parachute = 0.0;

            // Parameter Setter
            // void setEngine(Engine engine);
            void setLengthCG(const InterpolateParameter length_CG);
            void setLengthCP(const InterpolateParameter length_CP);
            void setCA(const InterpolateParameter CA);
            void setCA(const InterpolateParameter CA, const InterpolateParameter CA_burnout);
            void setCNa(const InterpolateParameter CNa);
            void setCld(const InterpolateParameter Cld);
            void setClp(const InterpolateParameter Clp);
            void setCmq(const InterpolateParameter Cmq);
            void setCnr(const InterpolateParameter Cnr);

            void setInertiaTensor(const InterpolateParameter MOI_xx, const InterpolateParameter MOI_yy, const InterpolateParameter MOI_zz);
            void setAttitudeProgram(const InterpolateParameter azimuth, const InterpolateParameter elevation, const InterpolateParameter roll);

            void setCdSParachute(const double CdS_first);
            void setCdSParachute(const double CdS_first, const double CdS_second);

            // Parameter Getter
            double getLengthCG();
            double getLengthCP(const double mach_number);
            double getCA(const double mach_number);
            double getCNa(const double mach_number);
            double getCld(const double mach_number);
            double getClp(const double mach_number);
            double getCmq(const double mach_number);
            double getCnr(const double mach_number);

            Eigen::Vector3d getThrust(const double air_pressure, const double air_pressure_sea_level);
            Eigen::Matrix3d getInertiaTensor();
            Eigen::Vector3d getAttitude();

            // SOE Handler
            void IgnitionEngine(DateTime UTC_init, double countup_time_init);
            void CutoffEngine();
            void DeSpin();
            void JettsonFairing(const double mass_fairing);
            void SeparateUpperStage(const double mass_upper_stage);

            void OpenParachute();

        private:

            InterpolateParameter inertia_moment_xx_src;
            InterpolateParameter inertia_moment_yy_src;
            InterpolateParameter inertia_moment_zz_src;

            InterpolateParameter azimuth_program_src;
            InterpolateParameter elevation_program_src;
            InterpolateParameter roll_program_src;

            InterpolateParameter length_CG_src;  // from end
            InterpolateParameter length_CP_src;  // from end
            InterpolateParameter CA_src;
            InterpolateParameter CA_burnout_src;
            InterpolateParameter CNa_src;
            InterpolateParameter Cld_src;
            InterpolateParameter Clp_src;
            InterpolateParameter Cmq_src;
            InterpolateParameter Cnr_src;

            int count_open_parachute = 0;
            std::vector<double> CdS_parachute_src;

    };

}


#endif