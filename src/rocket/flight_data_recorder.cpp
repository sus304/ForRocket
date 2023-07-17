// ******************************************************
// Project Name    : ForRocket
// File Name       : flight_data_recorder.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "flight_data_recorder.hpp"

#include <iostream>
#include <iomanip>

#include "degrad.hpp"
#include "environment/air.hpp"
#include "environment/gravity.hpp"
#include "environment/vincenty.hpp"
#include "dynamics/noniterative_iip.hpp"


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
    for (int i=0; i < countup_burn_time.size(); ++i) {
        double t = countup_burn_time[i];
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
            ofs << std::setprecision(8) << rad2deg(attitude[i].euler_angle(0)) << ",";  // [deg]
            ofs << std::setprecision(8) << rad2deg(attitude[i].euler_angle(1)) << ",";  // [deg]
            ofs << std::setprecision(8) << rad2deg(attitude[i].euler_angle(2)) << ",";  // [deg]
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





