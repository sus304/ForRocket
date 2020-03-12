// ******************************************************
// Project Name    : ForRocket
// File Name       : datetime.hpp
// Creation Date   : 2019/11/08
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef DATETIME_HPP_
#define DATETIME_HPP_

#include <string>

namespace forrocket {
    class DateTime {
        public:
            unsigned int year;
            unsigned int month;
            unsigned int day;

            unsigned int hour;
            unsigned int min;
            unsigned int sec;
            unsigned int msec;

            DateTime();
            DateTime(unsigned int year, unsigned int month, unsigned int day, unsigned int hour);
            DateTime(unsigned int year, unsigned int month, unsigned int day, unsigned int hour, unsigned int min);
            DateTime(unsigned int year, unsigned int month, unsigned int day, unsigned int hour, unsigned int min, unsigned int sec);
            DateTime(unsigned int year, unsigned int month, unsigned int day, unsigned int hour, unsigned int min, unsigned int sec, unsigned int msec);
            DateTime(std::string datetime_str);

            DateTime(const DateTime& from);
            DateTime& operator=(const DateTime& from);

            DateTime operator+(const double sec);

    };

}

#endif
