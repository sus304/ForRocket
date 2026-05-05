// ******************************************************
// Project Name    : ForRocket
// File Name       : flight_event.hpp
// Creation Date   : 2026/05/02
//
// Copyright (c) 2026 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef FLIGHTEVENT_HPP_
#define FLIGHTEVENT_HPP_

#include <functional>

#include "dynamics/dynamics_base.hpp"
#include "environment/sequence_clock.hpp"
#include "rocket/rocket.hpp"

namespace forrocket {

class FlightEvent {
 public:
    virtual ~FlightEvent() {}
    virtual double Time() const = 0;
    virtual void Apply(Rocket& rocket, DynamicsBase::state& x) const = 0;
};


class IgnitionEvent : public FlightEvent {
 public:
    IgnitionEvent(double time, const SequenceClock* master_clock);
    double Time() const override { return time_; }
    void Apply(Rocket& rocket, DynamicsBase::state& x) const override;

 private:
    double time_;
    const SequenceClock* p_master_clock_;
};


class CutoffEvent : public FlightEvent {
 public:
    explicit CutoffEvent(double time) : time_(time) {}
    double Time() const override { return time_; }
    void Apply(Rocket& rocket, DynamicsBase::state& x) const override;

 private:
    double time_;
};


class DespinEvent : public FlightEvent {
 public:
    explicit DespinEvent(double time) : time_(time) {}
    double Time() const override { return time_; }
    void Apply(Rocket& rocket, DynamicsBase::state& x) const override;

 private:
    double time_;
};


class JettisonFairingEvent : public FlightEvent {
 public:
    JettisonFairingEvent(double time, double mass_fairing)
        : time_(time), mass_fairing_(mass_fairing) {}
    double Time() const override { return time_; }
    void Apply(Rocket& rocket, DynamicsBase::state& x) const override;

 private:
    double time_;
    double mass_fairing_;
};


class SeparationEvent : public FlightEvent {
 public:
    typedef std::function<void(double, const DynamicsBase::state&)> Callback;

    SeparationEvent(double time, double mass_upper_stage, Callback on_separate)
        : time_(time), mass_upper_stage_(mass_upper_stage), on_separate_(on_separate) {}
    double Time() const override { return time_; }
    void Apply(Rocket& rocket, DynamicsBase::state& x) const override;

 private:
    double time_;
    double mass_upper_stage_;
    Callback on_separate_;
};


class ParachuteOpenEvent : public FlightEvent {
 public:
    explicit ParachuteOpenEvent(double time) : time_(time) {}
    double Time() const override { return time_; }
    void Apply(Rocket& rocket, DynamicsBase::state& x) const override;

 private:
    double time_;
};

}  // namespace forrocket

#endif
