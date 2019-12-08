// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket.hpp"

#include <cmath>

forrocket::Rocket::Rocket() {
    length_thrust = 0.0;
    diameter = 0.0;
    area = 0.0;
    length = 0.0;
    length_CG = 0.0;
    length_CP = 0.0;
    inertia_tensor << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    CA = 0.0;
    CNa = 0.0;
    Clp = 0.0;
    Cmq = 0.0;
    Cnr = 0.0;

    cant_angle_fin = 0.0;
    CNa_fin = 0.0;
    radius_CP_fin = 0.0;

    dynamic_pressure = 0.0;

    quaternion_dot << 0.0, 0.0, 0.0, 0.0;
    angular_velocity << 0.0, 0.0, 0.0;
    angular_acceleration << 0.0, 0.0, 0.0;
    angle_of_attack = 0.0;
    sideslip_angle = 0.0;

};


void forrocket::Rocket::setLengthCG(const double length_CG) {
    enable_length_CG_from_log = false;    
    this->length_CG = length_CG;
};

void forrocket::Rocket::setLengthCG(const std::vector<double> time_vector, const std::vector<double> CG_vector) {
    enable_length_CG_from_log = true;    
    length_CG_polate = interpolate::Interp1d(time_vector, CG_vector, "linear", "same");
};

void forrocket::Rocket::UpdateLengthCG(const double t) {
    if (enable_length_CG_from_log) {
        length_CG = length_CG_polate(t);
    }
};


void forrocket::Rocket::UpdateAerodynamicsParameter(const double mach_number) {
    length_CP = length_CP_src(mach_number);
    CA = CA_src(mach_number);
    CNa = CNa_src(mach_number);
    Clp = Clp_src(mach_number);
    Cmq = Cmq_src(mach_number);
    Cnr = Cnr_src(mach_number);
};

void forrocket::Rocket::UpdateAerodynamicsParameter(const double mach_number, const double AoA) {
};


void forrocket::Rocket::UpdateAttitudeFromProgramRate(const double t) {

};


void forrocket::Rocket::setLengthCP(const AerodynamicsParameter length_CP) {
    this->length_CP_src = length_CP;
};

void forrocket::Rocket::setCA(const AerodynamicsParameter CA) {
    this->CA_src = CA;
};

void forrocket::Rocket::setCNa(const AerodynamicsParameter CNa) {
    this->CNa_src = CNa;
};

void forrocket::Rocket::setClp(const AerodynamicsParameter Clp) {
    this->Clp_src = Clp;
};

void forrocket::Rocket::setCmq(const AerodynamicsParameter Cmq) {
    this->Cmq_src = Cmq;
};

void forrocket::Rocket::setCnr(const AerodynamicsParameter Cnr) {
    this->Cnr_src = Cnr;
};

