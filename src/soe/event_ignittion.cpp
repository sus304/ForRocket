// ******************************************************
// Project Name    : ForRocket
// File Name       : event_ignittion.cpp
// Creation Date   : 2022/06/19

// Copyright © 2022 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "soe/event_ignittion.hpp"

forrocket::EventEngineIgnittion::EventEngineIgnittion(Rocket* p_rocket, double time_ignittion) {
    this->time_ignittion = time_ignittion;
    this->p_rocket = p_rocket;
    this->enable = true;
};

void forrocket::EventEngineIgnittion::TryEvent(const SequenceClock* clock) {
    if (enable && clock->countup_time >= time_ignittion) {
        p_rocket->IgnitionEngine(clock->UTC_date_init, clock->countup_time);
        enable = false;
    }
};

