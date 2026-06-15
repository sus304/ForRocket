// ******************************************************
// Project Name    : ForRocket
// File Name       : flight_data_recorder.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "flight_data_recorder.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>

#include "degrad.hpp"
#include "environment/air.hpp"
#include "environment/gravity.hpp"
#include "environment/vincenty.hpp"


forrocket::FlightDataRecorder::FlightDataRecorder(Rocket* rocket) {
    p_rocket = rocket;
};


void forrocket::FlightDataRecorder::ReserveCapacity(const int capacity) {
    countup_time.reserve(capacity);
    countup_burn_time.reserve(capacity);
    thrust.reserve(capacity);
    mdot_prop.reserve(capacity);
    burning.reserve(capacity);
    gimbal_angle_y_axis.reserve(capacity);
    gimbal_angle_z_axis.reserve(capacity);
    mass_prop.reserve(capacity);
    mass.reserve(capacity);
    length_CG.reserve(capacity);
    length_CP.reserve(capacity);
    inertia_tensor.reserve(capacity);
    CA.reserve(capacity);
    CNa.reserve(capacity);
    Cld.reserve(capacity);
    Clp.reserve(capacity);
    Cmq.reserve(capacity);
    position.reserve(capacity);
    velocity.reserve(capacity);
    dynamic_pressure.reserve(capacity);
    acceleration.reserve(capacity);
    force.reserve(capacity);
    attitude.reserve(capacity);
    quaternion_dot.reserve(capacity);
    angular_velocity.reserve(capacity);
    angular_acceleration.reserve(capacity);
    angle_of_attack.reserve(capacity);
    sideslip_angle.reserve(capacity);
    moment.reserve(capacity);
};


void forrocket::FlightDataRecorder::operator()(const DynamicsBase::state& x, const double t) {
    if (p_rocket->position.LLH(2) >= 0.0) {
        countup_time.push_back(t);
        countup_burn_time.push_back(p_rocket->burn_clock.countup_time);
        thrust.push_back(p_rocket->engine.thrust);
        mdot_prop.push_back(p_rocket->engine.mdot_prop);
        burning.push_back(p_rocket->engine.burning);
        gimbal_angle_y_axis.push_back(p_rocket->engine.gimbal_angle_y_axis);
        gimbal_angle_z_axis.push_back(p_rocket->engine.gimbal_angle_z_axis);
        mass_prop.push_back(p_rocket->mass.propellant);
        mass.push_back(p_rocket->mass.inert + p_rocket->mass.propellant);
        length_CG.push_back(p_rocket->length_CG);
        length_CP.push_back(p_rocket->length_CP);
        inertia_tensor.push_back(p_rocket->inertia_tensor);
        CA.push_back(p_rocket->CA);
        CNa.push_back(p_rocket->CNa);
        Cld.push_back(p_rocket->Cld);
        Clp.push_back(p_rocket->Clp);
        Cmq.push_back(p_rocket->Cmq);
        position.push_back(p_rocket->position);
        velocity.push_back(p_rocket->velocity);
        dynamic_pressure.push_back(p_rocket->dynamic_pressure);
        acceleration.push_back(p_rocket->acceleration);
        force.push_back(p_rocket->force);
        attitude.push_back(p_rocket->attitude);
        quaternion_dot.push_back(p_rocket->quaternion_dot);
        angular_velocity.push_back(p_rocket->angular_velocity);
        angular_acceleration.push_back(p_rocket->angular_acceleration);
        angle_of_attack.push_back(p_rocket->angle_of_attack);
        sideslip_angle.push_back(p_rocket->sideslip_angle);
        moment.push_back(p_rocket->moment);
    }
};


void forrocket::FlightDataRecorder::DumpCsv(const std::string file_name, bool full_dump) {
    std::ofstream ofs(file_name, std::ios::out);

    ofs << "Time [s],";
    ofs << "Burn Time [s],";
    if (full_dump) {
        ofs << "AirDensity [kg/m3],";
        ofs << "AirPressure [kPa],";
        ofs << "AirTemprature [K],";
        ofs << "SpeedOfSound [m/s],";
        ofs << "Propellant Mass [kg],";
        ofs << "Mass [kg],";
        ofs << "X-C.G. [%],";
        ofs << "X-C.P. [%],";
        ofs << "StaticMargin [%],";
        ofs << "xx_InertiaTensor [kg-m2],";
        ofs << "xy_InertiaTensor [kg-m2],";
        ofs << "xz_InertiaTensor [kg-m2],";
        ofs << "yx_InertiaTensor [kg-m2],";
        ofs << "yy_InertiaTensor [kg-m2],";
        ofs << "yz_InertiaTensor [kg-m2],";
        ofs << "zx_InertiaTensor [kg-m2],";
        ofs << "zy_InertiaTensor [kg-m2],";
        ofs << "zz_InertiaTensor [kg-m2],";
        ofs << "Thrust [N],";
        ofs << "Mdot [kg/s],";
        ofs << "Burning [0/1],";
        ofs << "y-Gimbal [deg],";
        ofs << "z-Gimbal [deg],";
        ofs << "CA [-],";
        ofs << "CNa [-],";
        ofs << "Cld [-],";
        ofs << "Clp [-],";
        ofs << "Cmq [-],";
        ofs << "Cma [-],";  // pitch moment coefficient
        ofs << "AoA [deg],";
        ofs << "AoS [deg],";
        ofs << "Fx-aero [N],";
        ofs << "Fy-aero [N],";
        ofs << "Fz-aero [N],";
        ofs << "Fx-thrust [N],";
        ofs << "Fy-thrust [N],";
        ofs << "Fz-thrust [N],";
        ofs << "Fx-gravity [N],";
        ofs << "Fy-gravity [N],";
        ofs << "Fz-gravity [N],";
        ofs << "Accx-body [m/s2],";
        ofs << "Accy-body [m/s2],";
        ofs << "Accz-body [m/s2],";
        ofs << "Gccx-body [G],";
        ofs << "Gccy-body [G],";
        ofs << "Gccz-body [G],";
        ofs << "Accx-ECI [m/s2],";
        ofs << "Accy-ECI [m/s2],";
        ofs << "Accz-ECI [m/s2],";
        ofs << "Vx-body [m/s],";
        ofs << "Vy-body [m/s],";
        ofs << "Vz-body [m/s],";
        ofs << "MachNumber [-],";
        ofs << "DynamicPressure [kPa],";
        ofs << "Vx-NED [m/s],";
        ofs << "Vy-NED [m/s],";
        ofs << "Vz-NED [m/s],";
        ofs << "Vx-ECEF [m/s],";
        ofs << "Vy-ECEF [m/s],";
        ofs << "Vz-ECEF [m/s],";
        ofs << "Vx-ECI [m/s],";
        ofs << "Vy-ECI [m/s],";
        ofs << "Vz-ECI [m/s],";
        ofs << "X-ECEF [km],";
        ofs << "Y-ECEF [km],";
        ofs << "Z-ECEF [km],";
        ofs << "X-ECI [km],";
        ofs << "Y-ECI [km],";
        ofs << "Z-ECI [km],";
    }
    ofs << "Latitude [deg],";
    ofs << "Longitude [deg],";
    ofs << "Altitude [m],";
    ofs << "Downrange [m],";
    if (full_dump) {
        ofs << "Mx-thrust [Nm],";
        ofs << "My-thrust [Nm],";
        ofs << "Mz-thrust [Nm],";
        ofs << "Mx-aero [Nm],";
        ofs << "My-aero [Nm],";
        ofs << "Mz-aero [Nm],";
        ofs << "Mx-aerodump [Nm],";
        ofs << "My-aerodump [Nm],";
        ofs << "Mz-aerodump [Nm],";
        ofs << "Mx-jetdump [Nm],";
        ofs << "My-jetdump [Nm],";
        ofs << "Mz-jetdump [Nm],";
        ofs << "Mx-gasjet [Nm],";
        ofs << "My-gasjet [Nm],";
        ofs << "Mz-gasjet [Nm],";
        ofs << "Mx-gyroeffect [Nm],";
        ofs << "My-gyroeffect [Nm],";
        ofs << "Mz-gyroeffect [Nm],";
        ofs << "Mx [Nm],";
        ofs << "My [Nm],";
        ofs << "Mz [Nm],";
        ofs << "AngleAccx [rad/s2],";
        ofs << "AngleAccy [rad/s2],";
        ofs << "AngleAccz [rad/s2],";
        ofs << "AngleVelx [deg/s],";
        ofs << "AngleVely [deg/s],";
        ofs << "AngleVelz [deg/s],";
        ofs << "q1 [-],";
        ofs << "q2 [-],";
        ofs << "q3 [-],";
        ofs << "q4 [-],";
        ofs << "Azimuth [deg],";
        ofs << "Elvation [deg],";
        ofs << "Roll [deg],";
        // ---- Roll-pitch (spin-pitch / roll-yaw) resonance diagnostics ----
        ofs << "PitchYawNaturalFreq [Hz],";
        ofs << "SpinFreq [Hz],";
        ofs << "ResonanceRatio [-],";
        ofs << "TotalAoA [deg],";
        ofs << "TrimAoA [deg],";
        ofs << "GyroStabilityFactor Sg [-],";
        ofs << "PitchDampingRatio [-],";
        ofs << "DynStabilityFactor Sd [-],";
        ofs << "DynStabilityBoundary Sd(2-Sd) [-],";
        ofs << "DynStable [0/1],";
        ofs << "ResonanceAmplification [-],";
        ofs << "EquilibriumSpinFreq [Hz],";
        ofs << "LateralAeroLoad [N],";
    }
    // pitch
    // yaw
    // nutation
    // if (full_dump) {
    //     ofs << "Time [s],";
    //     ofs << "Time of IIP [s],";
    //     ofs << "Latitude of IIP [deg],";
    //     ofs << "Longitude of IIP [deg],";
    //     ofs << "Downrange of IIP [m],";
    // }

    ofs << std::endl;

    double g0 = gravity(0.0);
    ofs << std::fixed;
    for (std::size_t i=0; i < countup_burn_time.size(); ++i) {
        double alt = position[i].LLH(2);
        EnvironmentAir air(alt);


        ofs << std::setprecision(4) << countup_time[i] << ",";
        ofs << std::setprecision(4) << countup_burn_time[i] << ",";
        if (full_dump) {
            ofs << std::setprecision(6) << air.density << ",";
            ofs << std::setprecision(6) << air.pressure / 1e3 << ",";  // [kPa]
            ofs << std::setprecision(6) << air.temprature << ",";  // [K]
            ofs << std::setprecision(6) << air.speed_of_sound << ",";
            ofs << std::setprecision(8) << mass_prop[i] << ",";
            ofs << std::setprecision(8) << mass[i] << ",";
            ofs << std::setprecision(2) << length_CG[i] / p_rocket->length * 100.0 << ",";  // [%]
            ofs << std::setprecision(2) << length_CP[i] / p_rocket->length * 100.0 << ",";  // [%]
            ofs << std::setprecision(2) << (length_CG[i] - length_CP[i]) / p_rocket->length * 100.0 << ",";  // Fst [%]
            ofs << std::setprecision(8) << inertia_tensor[i](0, 0) << ",";
            ofs << std::setprecision(8) << inertia_tensor[i](0, 1) << ",";
            ofs << std::setprecision(8) << inertia_tensor[i](0, 2) << ",";
            ofs << std::setprecision(8) << inertia_tensor[i](1, 0) << ",";
            ofs << std::setprecision(8) << inertia_tensor[i](1, 1) << ",";
            ofs << std::setprecision(8) << inertia_tensor[i](1, 2) << ",";
            ofs << std::setprecision(8) << inertia_tensor[i](2, 0) << ",";
            ofs << std::setprecision(8) << inertia_tensor[i](2, 1) << ",";
            ofs << std::setprecision(8) << inertia_tensor[i](2, 2) << ",";
            ofs << std::setprecision(8) << thrust[i] << ",";
            ofs << std::setprecision(8) << mdot_prop[i] << ",";
            ofs << burning[i] << ",";
            ofs << std::setprecision(8) << rad2deg(gimbal_angle_y_axis[i]) << ",";  // [deg]
            ofs << std::setprecision(8) << rad2deg(gimbal_angle_z_axis[i]) << ",";  // [deg]
            ofs << std::setprecision(4) << CA[i] << ",";
            ofs << std::setprecision(4) << CNa[i] << ",";
            ofs << std::setprecision(4) << Cld[i] << ",";
            ofs << std::setprecision(4) << Clp[i] << ",";
            ofs << std::setprecision(4) << Cmq[i] << ",";
            ofs << std::setprecision(4) << CNa[i] * (length_CG[i] - length_CP[i]) / p_rocket->length << ",";
            ofs << std::setprecision(8) << rad2deg(angle_of_attack[i]) << ",";  // [deg]
            ofs << std::setprecision(8) << rad2deg(sideslip_angle[i]) << ",";  // [deg]
            ofs << std::setprecision(8) << force[i].aero(0) << ",";
            ofs << std::setprecision(8) << force[i].aero(1) << ",";
            ofs << std::setprecision(8) << force[i].aero(2) << ",";
            ofs << std::setprecision(8) << force[i].thrust(0) << ",";
            ofs << std::setprecision(8) << force[i].thrust(1) << ",";
            ofs << std::setprecision(8) << force[i].thrust(2) << ",";
            ofs << std::setprecision(8) << force[i].gravity(0) << ",";
            ofs << std::setprecision(8) << force[i].gravity(1) << ",";
            ofs << std::setprecision(8) << force[i].gravity(2) << ",";
            ofs << std::setprecision(8) << acceleration[i].body(0) << ",";  // [m/s2]
            ofs << std::setprecision(8) << acceleration[i].body(1) << ",";  // [m/s2]
            ofs << std::setprecision(8) << acceleration[i].body(2) << ",";  // [m/s2]
            ofs << std::setprecision(8) << acceleration[i].body(0) / g0 << ",";  // [G]
            ofs << std::setprecision(8) << acceleration[i].body(1) / g0 << ",";  // [G]
            ofs << std::setprecision(8) << acceleration[i].body(2) / g0 << ",";  // [G]
            ofs << std::setprecision(8) << acceleration[i].ECI(0) << ",";
            ofs << std::setprecision(8) << acceleration[i].ECI(1) << ",";
            ofs << std::setprecision(8) << acceleration[i].ECI(2) << ",";
            ofs << std::setprecision(8) << velocity[i].air_body(0) << ",";
            ofs << std::setprecision(8) << velocity[i].air_body(1) << ",";
            ofs << std::setprecision(8) << velocity[i].air_body(2) << ",";
            ofs << std::setprecision(8) << velocity[i].mach_number << ",";
            ofs << std::setprecision(8) << dynamic_pressure[i] / 1e3 << ",";  // [kPa]
            ofs << std::setprecision(8) << velocity[i].NED(0) << ",";
            ofs << std::setprecision(8) << velocity[i].NED(1) << ",";
            ofs << std::setprecision(8) << velocity[i].NED(2) << ",";
            ofs << std::setprecision(8) << velocity[i].ECEF(0) << ",";
            ofs << std::setprecision(8) << velocity[i].ECEF(1) << ",";
            ofs << std::setprecision(8) << velocity[i].ECEF(2) << ",";
            ofs << std::setprecision(8) << velocity[i].ECI(0) << ",";
            ofs << std::setprecision(8) << velocity[i].ECI(1) << ",";
            ofs << std::setprecision(8) << velocity[i].ECI(2) << ",";
            ofs << std::setprecision(10) << position[i].ECEF(0) / 1e3 << ",";  // [km]
            ofs << std::setprecision(10) << position[i].ECEF(1) / 1e3 << ",";  // [km]
            ofs << std::setprecision(10) << position[i].ECEF(2) / 1e3 << ",";  // [km]
            ofs << std::setprecision(10) << position[i].ECI(0) / 1e3 << ",";  // [km]
            ofs << std::setprecision(10) << position[i].ECI(1) / 1e3 << ",";  // [km]
            ofs << std::setprecision(10) << position[i].ECI(2) / 1e3 << ",";  // [km]
        }
        ofs << std::setprecision(8) << position[i].LLH(0) << ",";
        ofs << std::setprecision(8) << position[i].LLH(1) << ",";
        ofs << std::setprecision(8) << position[i].LLH(2) << ",";
        ofs << std::setprecision(8) << vdownrange(position[0].LLH, position[i].LLH).first << ",";
        if (full_dump) {
            ofs << std::setprecision(8) << moment[i].thrust(0) << ",";
            ofs << std::setprecision(8) << moment[i].thrust(1) << ",";
            ofs << std::setprecision(8) << moment[i].thrust(2) << ",";
            ofs << std::setprecision(8) << moment[i].aero_force(0) << ",";
            ofs << std::setprecision(8) << moment[i].aero_force(1) << ",";
            ofs << std::setprecision(8) << moment[i].aero_force(2) << ",";
            ofs << std::setprecision(8) << moment[i].aero_dumping(0) << ",";
            ofs << std::setprecision(8) << moment[i].aero_dumping(1) << ",";
            ofs << std::setprecision(8) << moment[i].aero_dumping(2) << ",";
            ofs << std::setprecision(8) << moment[i].jet_dumping(0) << ",";
            ofs << std::setprecision(8) << moment[i].jet_dumping(1) << ",";
            ofs << std::setprecision(8) << moment[i].jet_dumping(2) << ",";
            ofs << std::setprecision(8) << moment[i].gas_jet(0) << ",";
            ofs << std::setprecision(8) << moment[i].gas_jet(1) << ",";
            ofs << std::setprecision(8) << moment[i].gas_jet(2) << ",";
            ofs << std::setprecision(8) << moment[i].gyro(0) << ",";
            ofs << std::setprecision(8) << moment[i].gyro(1) << ",";
            ofs << std::setprecision(8) << moment[i].gyro(2) << ",";
            ofs << std::setprecision(8) << moment[i].Sum()(0) << ",";
            ofs << std::setprecision(8) << moment[i].Sum()(1) << ",";
            ofs << std::setprecision(8) << moment[i].Sum()(2) << ",";
            ofs << std::setprecision(8) << angular_acceleration[i](0) << ",";  // [rad/s2]
            ofs << std::setprecision(8) << angular_acceleration[i](1) << ",";  // [rad/s2]
            ofs << std::setprecision(8) << angular_acceleration[i](2) << ",";  // [rad/s2]
            ofs << std::setprecision(8) << rad2deg(angular_velocity[i](0)) << ",";  // [deg/s]
            ofs << std::setprecision(8) << rad2deg(angular_velocity[i](1)) << ",";  // [deg/s]
            ofs << std::setprecision(8) << rad2deg(angular_velocity[i](2)) << ",";  // [deg/s]
            ofs << std::setprecision(8) << attitude[i].quaternion(0) << ",";
            ofs << std::setprecision(8) << attitude[i].quaternion(1) << ",";
            ofs << std::setprecision(8) << attitude[i].quaternion(2) << ",";
            ofs << std::setprecision(8) << attitude[i].quaternion(3) << ",";
            ofs << std::setprecision(8) << std::fmod(rad2deg(attitude[i].euler_angle(0)) + 360.0, 360.0) << ",";  // [deg] [0, 360)
            ofs << std::setprecision(8) << rad2deg(attitude[i].euler_angle(1)) << ",";  // [deg]
            ofs << std::setprecision(8) << rad2deg(attitude[i].euler_angle(2)) << ",";  // [deg]

            // ---- Roll-pitch (spin-pitch / roll-yaw) resonance diagnostics ----
            const double q_dyn = dynamic_pressure[i];                       // [Pa]
            const double S_ref = p_rocket->area;                            // [m2]
            const double D_ref = p_rocket->diameter;                        // [m]
            const double Vair  = velocity[i].air_body.norm();               // airspeed [m/s]
            const double Ix    = inertia_tensor[i](0, 0);                   // roll MOI [kg-m2]
            const double It    = 0.5 * (inertia_tensor[i](1, 1) + inertia_tensor[i](2, 2));  // transverse MOI (avg Iyy,Izz) [kg-m2]
            const double sm    = length_CG[i] - length_CP[i];               // static margin distance [m] (>0 : statically stable)
            const double pspin = angular_velocity[i](0);                    // roll(spin) rate [rad/s]
            const double m_now = mass[i];                                   // total mass [kg]

            // pitch/yaw aerodynamic restoring stiffness  k_alpha = q*S*CNa*(Xcg-Xcp)  [N-m/rad]
            const double k_alpha = q_dyn * S_ref * CNa[i] * sm;
            const double omega_n = (k_alpha > 0.0 && It > 0.0) ? std::sqrt(k_alpha / It) : 0.0;  // natural freq [rad/s]
            const double f_n     = omega_n / (2.0 * pi);                    // [Hz]
            const double f_spin  = std::abs(pspin) / (2.0 * pi);            // [Hz]
            const double lambda  = (omega_n > 0.0) ? std::abs(pspin) / omega_n : 0.0;  // resonance ratio (~1 : resonance)

            // total angle of attack [rad]
            const double aoa_tot = std::sqrt(angle_of_attack[i] * angle_of_attack[i]
                                           + sideslip_angle[i]  * sideslip_angle[i]);

            // pitch/yaw transverse damping coefficient [N-m/(rad/s)], two contributions:
            //   (1) lift damping from CP-CG offset (the dominant term for finned bodies):  q*S*CNa*sm^2/V
            //   (2) aerodynamic pitch-damping moment (Cmq, <0 -> stabilizing):            -q*S*D^2*Cmq/(2V)
            const double c_lift = (Vair > 0.0) ? q_dyn * S_ref * CNa[i] * sm * sm / Vair : 0.0;
            const double c_damp = (Vair > 0.0) ? -q_dyn * Cmq[i] * S_ref * D_ref * D_ref / (2.0 * Vair) : 0.0;
            const double zeta   = (k_alpha > 0.0 && It > 0.0) ? (c_lift + c_damp) / (2.0 * std::sqrt(k_alpha * It)) : 0.0;
            const double Qamp   = (std::abs(zeta) > 1.0e-9) ? 1.0 / (2.0 * std::abs(zeta)) : 0.0;  // resonance amplification ~1/(2*zeta)

            // gyroscopic stability factor  Sg = (Ix*p)^2 / (4*It*k_alpha)  (>1 : gyroscopically stable)
            const double Sg = (k_alpha > 0.0 && It > 0.0) ? std::pow(Ix * pspin, 2) / (4.0 * It * k_alpha) : 0.0;

            // dynamic stability factor (McCoy form); Magnus (Cmpa) and Cmadot are NOT modeled -> taken as 0
            const double inv_ky2  = (It > 0.0) ? m_now * D_ref * D_ref / It : 0.0;  // 1/k_y^2 = m*d^2/It
            const double sd_denom = CNa[i] - CA[i] - inv_ky2 * Cmq[i];
            const double Sd       = (std::abs(sd_denom) > 1.0e-12) ? 2.0 * CNa[i] / sd_denom : 0.0;
            const double sd_bound = Sd * (2.0 - Sd);
            const int dyn_stable  = (Sg > 0.0 && (1.0 / Sg) < sd_bound) ? 1 : 0;

            // equilibrium spin rate from fin-cant drive vs roll damping balance:  p_eq = -Cld*delta*2V/(Clp*D)
            const double delta_fin = p_rocket->cant_angle_fin;
            const double p_eq      = (std::abs(Clp[i]) > 1.0e-12 && Vair > 0.0)
                                     ? -Cld[i] * delta_fin * 2.0 * Vair / (Clp[i] * D_ref) : 0.0;
            const double f_spin_eq = std::abs(p_eq) / (2.0 * pi);           // [Hz]

            // lateral aerodynamic load proxy [N]
            const double lat_load = q_dyn * CNa[i] * S_ref * aoa_tot;

            // trim angle of attack from configurational-asymmetry forcing, with resonance amplification:
            //   forcing = transverse thrust-offset moment + principal-axis-misalignment (products of inertia) moment
            const double M_thrust_lat = std::sqrt(moment[i].thrust(1) * moment[i].thrust(1)
                                                + moment[i].thrust(2) * moment[i].thrust(2));
            const double I_offdiag    = std::sqrt(inertia_tensor[i](0, 1) * inertia_tensor[i](0, 1)
                                                + inertia_tensor[i](0, 2) * inertia_tensor[i](0, 2));
            const double M_poi        = pspin * pspin * I_offdiag;
            const double M_asym       = M_thrust_lat + M_poi;
            const double amp_resp     = std::sqrt((1.0 - lambda * lambda) * (1.0 - lambda * lambda)
                                                + (2.0 * zeta * lambda) * (2.0 * zeta * lambda));
            const double alpha_trim   = (k_alpha > 0.0 && amp_resp > 1.0e-9) ? (M_asym / (k_alpha * amp_resp)) : 0.0;  // [rad]

            ofs << std::setprecision(8) << f_n << ",";
            ofs << std::setprecision(8) << f_spin << ",";
            ofs << std::setprecision(8) << lambda << ",";
            ofs << std::setprecision(8) << rad2deg(aoa_tot) << ",";    // [deg]
            ofs << std::setprecision(8) << rad2deg(alpha_trim) << ",";  // [deg]
            ofs << std::setprecision(8) << Sg << ",";
            ofs << std::setprecision(8) << zeta << ",";
            ofs << std::setprecision(8) << Sd << ",";
            ofs << std::setprecision(8) << sd_bound << ",";
            ofs << dyn_stable << ",";
            ofs << std::setprecision(8) << Qamp << ",";
            ofs << std::setprecision(8) << f_spin_eq << ",";
            ofs << std::setprecision(8) << lat_load << ",";
        }
        // pitch = omega
        // pitch
        // yaw
        // nutation
        // flight path angle
        // 軌道情報

        // if (full_dump) {
        //     ofs << std::setprecision(4) << countup_burn_time[i] << ",";
        //     auto iip = IIP(position[i].ECI, velocity[i].ECI);
        //     ofs << std::setprecision(4) << iip.first << ",";  // [s]
        //     ofs << std::setprecision(8) << iip.second(0) << ",";  // [deg]
        //     ofs << std::setprecision(8) << iip.second(1) << ",";  // [deg]
        //     ofs << std::setprecision(8) << vdownrange(position[0].LLH, iip.second).first << ",";
        // }

        ofs << std::endl;
    }
    ofs.close();
};





