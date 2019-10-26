// ******************************************************
// Project Name    : ForRocket
// File Name       : engine.cpp
// Creation Date   : 2019/10/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "engine.hpp"

void forrocket::Engine::Ingnition() {
    thrust = thrust_source;
    mdot_prop = mdot_prop_source;
};


void forrocket::Engine::Cutoff() {
    thrust = 0.0;
    mdot_prop = 0.0;
};

