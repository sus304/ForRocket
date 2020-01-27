// ******************************************************
// Project Name    : ForRocket
// File Name       : sequence_clock.hpp
// Creation Date   : 2019/11/08
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef SEQUENCECLOCK_HPP_
#define SEQUENCECLOCK_HPP_

#include "environment/datetime.hpp"

namespace forrocket {
    class SequenceClock {
        public:
            SequenceClock() {};
            SequenceClock(DateTime UTC_init);
            SequenceClock(DateTime UTC_init, double countup_time_init);

            void SyncSolverTime(const double t);

            double julian_data;  // ユリウス日
            double modified_julian_date;  // 準ユリウス日
            double greenwich_sidereal_time;  // グリニッジ恒星時
            // double local_sidereal_time;  // 地域恒星時
            DateTime UTC_date_init;
            double countup_time;  // カウントアップタイマー

        private:
            double countup_time_ref;  // カウントアップタイマー基準タイム

            void UpdateJulianDate();

            double UTC2JulianDate(DateTime UTC);
            double JulianDate2GreenwichSiderealTime(const double julian);
            double UTC2GreenwichSiderealTime(DateTime UTC);
            double JulianDate2ModifiedJulianDate(const double julian);
            DateTime ModifiedJulianDate2UTC(const double modified_julian);
    };

}

#endif
