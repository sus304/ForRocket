// ******************************************************
// Project Name    : ForRocket
// File Name       : sequence_of_event.hpp
// Creation Date   : 2022/06/19
//
// Copyright (c) 2022 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef SEQUENCEOFEVENT_HPP_
#define SEQUENCEOFEVENT_HPP_

#include "rocket/rocket.hpp"

namespace forrocket {

class SequenceOfEvent {
    public:
        virtual void TryEvent() = 0;
    protected:
        bool enable;
        Rocket* p_rocket;
};

}  // namespace forrocket

#endif