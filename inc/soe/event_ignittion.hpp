// ******************************************************
// Project Name    : ForRocket
// File Name       : event_ignittion.hpp
// Creation Date   : 2022/06/19
//
// Copyright (c) 2022 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef EVENTIGNITTION_HPP_
#define EVENTIGNITTION_HPP_

#include "soe/sequence_of_event.hpp"
#include "rocket/rocket.hpp"
#include "environment/sequence_clock.hpp"

namespace forrocket {

class EventEngineIgnittion : public SequenceOfEvent {
    public:
        EventEngineIgnittion(Rocket* p_rocket, const double time_ignittion);

        void TryEvent(const SequenceClock* clock);

    private:
        double time_ignittion;
};

}  // namespace forrocket

#endif