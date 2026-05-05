// ******************************************************
// Project Name    : ForRocket
// File Name       : flight_event.cpp
// Creation Date   : 2026/05/02
//
// Copyright (c) 2026 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "flight_event.hpp"


forrocket::IgnitionEvent::IgnitionEvent(double time, const SequenceClock* master_clock)
    : time_(time), p_master_clock_(master_clock) {}

void forrocket::IgnitionEvent::Apply(Rocket& rocket, DynamicsBase::state& x) const {
    rocket.IgnitionEngine(p_master_clock_->UTC_date_init, time_);
}


void forrocket::CutoffEvent::Apply(Rocket& rocket, DynamicsBase::state& x) const {
    rocket.CutoffEngine();
}


void forrocket::DespinEvent::Apply(Rocket& rocket, DynamicsBase::state& x) const {
    rocket.DeSpin();
    // roll角速度（state[10]）を直接ゼロに（以後の積分で連続）
    x[10] = 0.0;
}


void forrocket::JettisonFairingEvent::Apply(Rocket& rocket, DynamicsBase::state& x) const {
    rocket.JettsonFairing(mass_fairing_);
}


void forrocket::SeparationEvent::Apply(Rocket& rocket, DynamicsBase::state& x) const {
    rocket.SeparateUpperStage(mass_upper_stage_);
    if (on_separate_) {
        on_separate_(time_, x);
    }
}


void forrocket::ParachuteOpenEvent::Apply(Rocket& rocket, DynamicsBase::state& x) const {
    rocket.OpenParachute();
}
