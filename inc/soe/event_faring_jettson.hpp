// ******************************************************
// Project Name    : ForRocket
// File Name       : event_fairing_jettson.hpp
// Creation Date   : 2022/06/21
//
// Copyright (c) 2022 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef EVENTFARINGJETTSON_HPP_
#define EVENTFARINGJETTSON_HPP_

#include "soe/sequence_of_event.hpp"
#include "rocket/rocket.hpp"
#include "environment/sequence_clock.hpp"

namespace forrocket {

class EventFairingJettson : public SequenceOfEvent {
    public:
        EventFairingJettson(Rocket* p_rocket, const double time_jettson, const double mass_fairing);

        void TryEvent(const SequenceClock* clock);

    private:
        double time_ignittion;
};

}  // namespace forrocket

#endif