// ******************************************************
// Project Name    : ForRocket
// File Name       : pylib.hpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef PYLIB_HPP_
#define PYLIB_HPP_

#include <vector>

namespace forrocket {
    template <class T> std::vector<T> arange(const T start, const T stop, const T step);
    template <class T> std::vector<T> linspace(const T start, const T stop, const int num);
}

#endif
