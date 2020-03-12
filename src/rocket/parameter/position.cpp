// ******************************************************
// Project Name    : ForRocket
// File Name       : position.cpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "position.hpp"

#include "environment/sequence_clock.hpp"

forrocket::Position::Position() {
    ECI << 0.0, 0.0, 0.0;
    ECEF << 0.0, 0.0, 0.0;
    LLH << 0.0, 0.0, 0.0;
};

void forrocket::Position::Initialize(const DateTime datetime, const Eigen::Vector3d& LLH) {
    SequenceClock clock(datetime);
    Coordinate coordinate;
    this->LLH = LLH;
    ECEF = coordinate.LLH2ECEF(this->LLH);
    coordinate.setECI2ECEF(clock.countup_time);
    ECI = coordinate.dcm.ECEF2ECI * ECEF;
};


void forrocket::Position::Update(Coordinate& coordinate, const Eigen::Vector3d& ECI) {
    this->ECI = ECI;
    this->ECEF = coordinate.dcm.ECI2ECEF * this->ECI;
    this->LLH = coordinate.ECEF2LLH(this->ECEF);
};
