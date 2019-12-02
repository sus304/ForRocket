// ******************************************************
// Project Name    : ForRocket
// File Name       : engine_factory.hpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ENGINEFACTORY_HPP_
#define ENGINEFACTORY_HPP_

#include "engine.hpp"

namespace forrocket {
    class EngineFactory {
        Engine Create();
    };
}

#endif
