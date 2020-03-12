// ******************************************************
// Project Name    : ForRocket
// File Name       : datetime.cpp
// Creation Date   : 2019/11/08
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "datetime.hpp"

#include <cmath>

#include "fileio.hpp"

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


forrocket::DateTime::DateTime(std::string datetime_str) {
    auto datetime_str_vec = split(datetime_str, ' ');
    auto date_str = datetime_str_vec[0];
    auto time_str = datetime_str_vec[1];

    auto date_str_vec = split(date_str, '/');
    year = std::stoi(date_str_vec[0]);
    month = std::stoi(date_str_vec[1]);
    day = std::stoi(date_str_vec[2]);

    auto time_str_vec = split(time_str, ':');
    hour = std::stoi(time_str_vec[0]);
    min = std::stoi(time_str_vec[1]);
    auto sec_str_vec = split(time_str_vec[2], '.');
    sec = std::stoi(sec_str_vec[0]);
    msec = std::stoi(sec_str_vec[1]);
};


forrocket::DateTime::DateTime(const DateTime& from) {
    year = from.year;
    month = from.month;
    day = from.day;
    hour = from.hour;
    min = from.min;
    sec = from.sec;
    msec = from.msec;
};

forrocket::DateTime& forrocket::DateTime::operator=(const DateTime& from) {
    if (this != &from) {
        year = from.year;
        month = from.month;
        day = from.day;
        hour = from.hour;
        min = from.min;
        sec = from.sec;
        msec = from.msec;
    }
    return *this;
};


forrocket::DateTime forrocket::DateTime::operator+(const double sec_double) {
    DateTime new_datetime = *this;
    unsigned int sec_int = static_cast<unsigned int>(sec_double);

    unsigned int ms = static_cast<unsigned int>((sec_double - sec_int) * 1000.0);
    new_datetime.msec += ms;
    if (new_datetime.msec >= 1000) {
        new_datetime.sec += 1;
        new_datetime.msec -= 1000;
    }

    new_datetime.sec += sec_int;
    if (new_datetime.sec >= 60) {
        unsigned int m = static_cast<unsigned int>(new_datetime.sec / 60);
        new_datetime.min += m;
        new_datetime.sec -= m * 60;
    }

    if (new_datetime.min >= 60) {
        unsigned int h = static_cast<unsigned int>(new_datetime.min / 60);
        new_datetime.hour += h;
        new_datetime.min -= h * 60;
    }

    if (new_datetime.hour >= 24) {
        unsigned int d = static_cast<unsigned int>(new_datetime.hour / 24);
        new_datetime.day += d;
        new_datetime.hour -= d * 24;
    }

    // TODO:Month update
    return new_datetime;
};

