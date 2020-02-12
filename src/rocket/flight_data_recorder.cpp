// ******************************************************
// Project Name    : ForRocket
// File Name       : flight_data_recorder.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "flight_data_recorder.hpp"

#include <iostream>
#include <fstream>

forrocket::FlightDataRecorder::FlightDataRecorder(Rocket* rocket) {
    p_rocket = rocket;
};


void forrocket::FlightDataRecorder::operator()(const DynamicsBase::state& x, const double t) {
    countup_burn_time.push_back(p_rocket->burn_clock.countup_time);
    thrust.push_back(p_rocket->engine.thrust);
    mdot_prop.push_back(p_rocket->engine.mdot_prop);
    burning.push_back(p_rocket->engine.burning);
    mass_prop.push_back(p_rocket->mass.propellant);
    mass.push_back(p_rocket->mass.inert + p_rocket->mass.propellant);
    length_CG.push_back(p_rocket->length_CG);
    length_CP.push_back(p_rocket->length_CP);
    CA.push_back(p_rocket->CA);
    CNa.push_back(p_rocket->CNa);
    Cld.push_back(p_rocket->Cld);
    Clp.push_back(p_rocket->Clp);
    Cmq.push_back(p_rocket->Cmq);
    postion.push_back(p_rocket->position);
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

};

void forrocket::FlightDataRecorder::dump_csv(const std::string file_name) {
    #ifdef DEBUG
        std::cout << "dump:" << this << std::endl;
    #endif
    
    std::ofstream ofs(file_name, std::ios::out);

    ofs << "Time [s],";
    ofs << "Thrust [N],";
    ofs << "Mdot_p [kg/s],";
    ofs << "Burning [0/1],";
    ofs << "Mass_p [kg],";
    ofs << "Mass [kg],";
    ofs << "LCG [m],";
    ofs << "LCP [m],";
    ofs << "CA [-],";
    ofs << "CNa [1/rad],";
    ofs << "Cld [1/rad],";
    ofs << "Clp [-],";
    ofs << "Cmq [-],";
    ofs << "X-ECEF [m],";
    ofs << "Y-ECEF [m],";
    ofs << "Z-ECEF [m],";
    ofs << "X-ECI [m],";
    ofs << "Y-ECI [m],";
    ofs << "Z-ECI [m],";
    ofs << "Lat [deg],";
    ofs << "Lon [deg],";
    ofs << "Altitude [m]," << std::endl;

    for (int i=0; i < countup_burn_time.size(); ++i) {
        ofs << countup_burn_time[i] << ",";
        ofs << thrust[i] << ",";
        ofs << mdot_prop[i] << ",";
        ofs << burning[i] << ",";
        ofs << mass_prop[i] << ",";
        ofs << mass[i] << ",";
        ofs << length_CG[i] << ",";
        ofs << length_CP[i] << ",";
        ofs << CA[i] << ",";
        ofs << CNa[i] << ",";
        ofs << Cld[i] << ",";
        ofs << Clp[i] << ",";
        ofs << Cmq[i] << ",";
        ofs << postion[i].ECEF(0) << ",";
        ofs << postion[i].ECEF(1) << ",";
        ofs << postion[i].ECEF(2) << ",";
        ofs << postion[i].ECI(0) << ",";
        ofs << postion[i].ECI(1) << ",";
        ofs << postion[i].ECI(2) << ",";
        ofs << postion[i].LLH(0) << ",";
        ofs << postion[i].LLH(1) << ",";
        ofs << postion[i].LLH(2) << ",";
        ofs << velocity[i].air_body(0) << ",";
        ofs << velocity[i].air_body(1) << ",";
        ofs << velocity[i].air_body(2) << ",";
        ofs << velocity[i].mach_number << ",";
        ofs << velocity[i].NED(0) << ",";
        ofs << velocity[i].NED(1) << ",";
        ofs << velocity[i].NED(2) << ",";
        ofs << velocity[i].ECEF(0) << ",";
        ofs << velocity[i].ECEF(1) << ",";
        ofs << velocity[i].ECEF(2) << ",";
        ofs << velocity[i].ECI(0) << ",";
        ofs << velocity[i].ECI(1) << ",";
        ofs << velocity[i].ECI(2) << ",";
        ofs << dynamic_pressure[i] << ",";
        ofs << acceleration[i].body(0) << ",";
        ofs << acceleration[i].body(1) << ",";
        ofs << acceleration[i].body(2) << ",";
        ofs << acceleration[i].ECI(0) << ",";
        ofs << acceleration[i].ECI(1) << ",";
        ofs << acceleration[i].ECI(2) << ",";
        ofs << force[i].gravity(0) << ",";
        ofs << force[i].gravity(1) << ",";
        ofs << force[i].gravity(2) << ",";
        ofs << force[i].thrust(0) << ",";
        ofs << force[i].thrust(1) << ",";
        ofs << force[i].thrust(2) << ",";
        ofs << force[i].aero(0) << ",";
        ofs << force[i].aero(1) << ",";
        ofs << force[i].aero(2) << ",";
        ofs << attitude[i].euler_angle(0) << ",";
        ofs << attitude[i].euler_angle(1) << ",";
        ofs << attitude[i].euler_angle(2) << ",";
        ofs << attitude[i].quaternion(0) << ",";
        ofs << attitude[i].quaternion(1) << ",";
        ofs << attitude[i].quaternion(2) << ",";
        ofs << attitude[i].quaternion(3) << ",";
        ofs << angular_velocity[i](0) << ",";
        ofs << angular_velocity[i](1) << ",";
        ofs << angular_velocity[i](2) << ",";
        ofs << angular_acceleration[i](0) << ",";
        ofs << angular_acceleration[i](1) << ",";
        ofs << angular_acceleration[i](2) << ",";
        ofs << angle_of_attack[i] << ",";
        ofs << sideslip_angle[i] << ",";
        ofs << moment[i].thrust(0) << ",";
        ofs << moment[i].thrust(1) << ",";
        ofs << moment[i].thrust(2) << ",";
        ofs << moment[i].aero_force(0) << ",";
        ofs << moment[i].aero_force(1) << ",";
        ofs << moment[i].aero_force(2) << ",";
        ofs << moment[i].aero_dumping(0) << ",";
        ofs << moment[i].aero_dumping(1) << ",";
        ofs << moment[i].aero_dumping(2) << ",";
        ofs << moment[i].jet_dumping(0) << ",";
        ofs << moment[i].jet_dumping(1) << ",";
        ofs << moment[i].jet_dumping(2) << ",";
        ofs << moment[i].gyro(0) << ",";
        ofs << moment[i].gyro(1) << ",";
        ofs << moment[i].gyro(2) << std::endl;
    }
};
