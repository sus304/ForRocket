// ******************************************************
// Project Name    : ForRocket
// File Name       : velocity.cpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "velocity.hpp"

#include "environment/sequence_clock.hpp"

forrocket::Velocity::Velocity() {
    ECI << 0.0, 0.0, 0.0;
    ECEF << 0.0, 0.0, 0.0;
    NED << 0.0, 0.0, 0.0;
    air_body << 0.0, 0.0, 0.0;
    mach_number = 0.0;
};


void forrocket::Velocity::Initialize(const DateTime datetime, const Eigen::Vector3d& NED, const Eigen::Vector3d& pos_LLH, const Eigen::Vector3d& pos_ECI) {
    SequenceClock clock(datetime);
    Coordinate coordinate;
    this->NED = NED;
    coordinate.setECEF2NED(pos_LLH);
    this->ECEF = coordinate.dcm.NED2ECEF * this->NED;
    coordinate.setECI2ECEF(clock.greenwich_sidereal_time);
    this->ECI = coordinate.dcm.ECEF2ECI * this->ECEF + coordinate.dcm.EarthRotate * pos_ECI;
};


void forrocket::Velocity::Update(Coordinate& coordinate, const Eigen::Vector3d& ECI, const Eigen::Vector3d& pos_ECI) {
    this->ECI = ECI;
    this->ECEF = coordinate.dcm.ECI2ECEF * this->ECI - coordinate.dcm.EarthRotate * pos_ECI;
    this->NED = coordinate.dcm.ECEF2NED * this->ECEF;
};

