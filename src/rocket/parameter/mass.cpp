// ******************************************************
// Project Name    : ForRocket
// File Name       : mass.cpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "mass.hpp"

double forrocket::Mass::Sum() const {
    return inert + propellant;
};