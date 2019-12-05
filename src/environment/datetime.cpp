// ******************************************************
// Project Name    : ForRocket
// File Name       : datetime.cpp
// Creation Date   : 2019/11/08
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "datetime.hpp"

#include <cmath>

forrocket::DateTime::DateTime() {
    year = 2000;
    month = 1;
    day = 1;
    hour = 12;
    min = 0;
    sec = 0;
    msec = 0;
};


forrocket::DateTime::DateTime(unsigned int year, unsigned int month, unsigned int day, unsigned int hour) {
    this->year = year;
    this->month = month;
    this->day = day;
    this->hour = hour;
    this->min = 0;
    this->sec = 0;
    this->msec = 0;
};


forrocket::DateTime::DateTime(unsigned int year, unsigned int month, unsigned int day, unsigned int hour, unsigned int min) {
    this->year = year;
    this->month = month;
    this->day = day;
    this->hour = hour;
    this->min = min;
    this->sec = 0;
    this->msec = 0;
};


forrocket::DateTime::DateTime(unsigned int year, unsigned int month, unsigned int day, unsigned int hour, unsigned int min, unsigned int sec) {
    this->year = year;
    this->month = month;
    this->day = day;
    this->hour = hour;
    this->min = min;
    this->sec = sec;
    this->msec = 0;
};


forrocket::DateTime::DateTime(unsigned int year, unsigned int month, unsigned int day, unsigned int hour, unsigned int min, unsigned int sec, unsigned int msec) {
    this->year = year;
    this->month = month;
    this->day = day;
    this->hour = hour;
    this->min = min;
    this->sec = sec;
    this->msec = msec;
};


forrocket::DateTime forrocket::DateTime::operator+(const double sec) {
    unsigned int sec_int = static_cast<unsigned int>(sec);

    unsigned int ms = static_cast<unsigned int>((sec - sec_int) * 1000.0);
    msec += ms;
    if (msec >= 1000) {
        this->sec += 1;
        msec -= 1000;
    }

    this->sec += sec_int;
    if (this->sec >= 60) {
        unsigned int m = static_cast<unsigned int>(this->sec / 60);
        this->min += m;
        this->sec -= m * 60;
    }

    if (min >= 60) {
        unsigned int h = static_cast<unsigned int>(min / 60);
        hour += h;
        min -= h * 60;
    }

    if (hour >= 24) {
        unsigned int d = static_cast<unsigned int>(hour / 24);
        day += d;
        hour -= d * 24;
    }

    // TODO:Month update

};

