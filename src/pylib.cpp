// ******************************************************
// Project Name    : ForRocket
// File Name       : pylib.cpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "pylib.hpp"

template <class T> std::vector<T> forrocket::arange(const T start, const T stop, const T step) {
    std::vector<T> array;
    T value = start;
    while (value <= stop) {
        array.push_back(value);
        value = value + step;
    }
    return array;
}

template <class T> std::vector<T> forrocket::linspace(const T start, const T stop, const int num) {
    std::vector<T> array;
    T step = (start - stop) / num;
    for (int i=0; i <= num; i++) {
        array.push_back(start + i * step);
    }
    return array;
}