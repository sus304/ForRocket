// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket.cpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket.hpp"

#include <cmath>


void forrocket::Rocket::UpdateLengthCG(const double t) {


};


void forrocket::Rocket::UpdateLengthCP() {


};


void forrocket::Rocket::UpdateInertiaTensor(const double t) {

};


void forrocket::Rocket::UpdateEngine(const double t, const double air_pressure, const double air_pressure_sea_level) {
    if (t > engine.burn_duration || mass.propellant <= 0.0) {
        engine.Cutoff();
    }
    else {
        engine.Update(t, air_pressure_sea_level, air_pressure);
    }
}


void forrocket::Rocket::UpdateAerodynamicsCoefficient() {
    // double CA;
    // double CNa;
    // double Clp;
    // double Cmq;
    // double Cnr;
};


void forrocket::Rocket::UpdateAttitudeFromProgramRate(const double t) {

};



