// ******************************************************
// Unit tests for src/environment/datetime.cpp
//   - DateTime constructors (default, partial, full, copy, assignment)
//   - DateTime::operator+(double seconds): time advance with carry/rollover
//     across ms -> sec -> min -> hour -> day -> month (leap-aware) -> year
//
// Reference strategy: independent calendar arithmetic. Each expected value
// is derived by hand from civil-time carry rules, not characterization.
//
// Decision points exercised (operator+):
//   D1  msec >= 1000          (true / false)
//   D2  sec  >= 60            (true / false)
//   D3  min  >= 60            (true / false)
//   D4  hour >= 24            (true / false)
//   D5  while(day > days_in_month)        loop runs / skipped
//   D6    month > 12 inside the while     (true / false)
//   D7  days_in_month: Feb leap vs non-leap; (year%4)&&(year%100)||(year%400)
// ******************************************************

#include <gtest/gtest.h>

#include "environment/datetime.hpp"

using forrocket::DateTime;

// --- Constructors ---------------------------------------------------------

TEST(DateTime, DefaultIsJ2000Noon) {
    // Default ctor documents the J2000 reference epoch: 2000-01-01 12:00:00.
    DateTime dt;
    EXPECT_EQ(dt.year, 2000u);
    EXPECT_EQ(dt.month, 1u);
    EXPECT_EQ(dt.day, 1u);
    EXPECT_EQ(dt.hour, 12u);
    EXPECT_EQ(dt.min, 0u);
    EXPECT_EQ(dt.sec, 0u);
    EXPECT_EQ(dt.msec, 0u);
}

TEST(DateTime, PartialConstructorsZeroFillTrailingFields) {
    DateTime ymdh(2021, 3, 4, 5);
    EXPECT_EQ(ymdh.min, 0u);
    EXPECT_EQ(ymdh.sec, 0u);
    EXPECT_EQ(ymdh.msec, 0u);

    DateTime ymdhm(2021, 3, 4, 5, 6);
    EXPECT_EQ(ymdhm.min, 6u);
    EXPECT_EQ(ymdhm.sec, 0u);
    EXPECT_EQ(ymdhm.msec, 0u);

    DateTime ymdhms(2021, 3, 4, 5, 6, 7);
    EXPECT_EQ(ymdhms.sec, 7u);
    EXPECT_EQ(ymdhms.msec, 0u);

    DateTime full(2021, 3, 4, 5, 6, 7, 8);
    EXPECT_EQ(full.msec, 8u);
}

TEST(DateTime, StringConstructorParsesFields) {
    // Format: "YYYY/MM/DD hh:mm:ss.fff"
    DateTime dt(std::string("2019/11/08 13:24:35.678"));
    EXPECT_EQ(dt.year, 2019u);
    EXPECT_EQ(dt.month, 11u);
    EXPECT_EQ(dt.day, 8u);
    EXPECT_EQ(dt.hour, 13u);
    EXPECT_EQ(dt.min, 24u);
    EXPECT_EQ(dt.sec, 35u);
    EXPECT_EQ(dt.msec, 678u);
}

TEST(DateTime, CopyAndAssignment) {
    DateTime a(2021, 3, 4, 5, 6, 7, 8);
    DateTime b(a);  // copy ctor
    EXPECT_EQ(b.year, a.year);
    EXPECT_EQ(b.msec, a.msec);

    DateTime c;
    c = a;          // copy assignment
    EXPECT_EQ(c.month, a.month);
    EXPECT_EQ(c.sec, a.sec);

    // Self-assignment guard (this != &from false branch).
    c = c;
    EXPECT_EQ(c.year, 2021u);
}

// --- operator+ : carry/rollover -------------------------------------------

// D1 true: fractional second turns into msec, no other carry.
TEST(DateTime, AddHalfSecond) {
    DateTime dt(2000, 1, 1, 12, 0, 0, 0);
    DateTime r = dt + 0.5;  // 500 ms
    EXPECT_EQ(r.msec, 500u);
    EXPECT_EQ(r.sec, 0u);
    EXPECT_EQ(r.hour, 12u);
}

// D1 true with carry into seconds: 0.5s + existing 600ms = 1100ms -> +1s,100ms
TEST(DateTime, MsecCarriesIntoSeconds) {
    DateTime dt(2000, 1, 1, 12, 0, 0, 600);
    DateTime r = dt + 0.5;  // adds 500 ms -> 1100 ms
    EXPECT_EQ(r.sec, 1u);
    EXPECT_EQ(r.msec, 100u);
}

// D2 true: seconds overflow into minutes (90 s = 1 min 30 s).
TEST(DateTime, AddNinetySeconds) {
    DateTime dt(2000, 1, 1, 12, 0, 0, 0);
    DateTime r = dt + 90.0;
    EXPECT_EQ(r.min, 1u);
    EXPECT_EQ(r.sec, 30u);
}

// D3 true: minutes overflow into hours (3600 s = 1 h).
TEST(DateTime, AddOneHour) {
    DateTime dt(2000, 1, 1, 12, 0, 0, 0);
    DateTime r = dt + 3600.0;
    EXPECT_EQ(r.hour, 13u);
    EXPECT_EQ(r.min, 0u);
    EXPECT_EQ(r.sec, 0u);
}

// D4 + D5 true: hours overflow into a new day (43200 s = 12 h from noon).
TEST(DateTime, AddTwelveHoursRollsToNextDay) {
    DateTime dt(2000, 1, 1, 12, 0, 0, 0);
    DateTime r = dt + 43200.0;  // -> 2000-01-02 00:00:00
    EXPECT_EQ(r.day, 2u);
    EXPECT_EQ(r.hour, 0u);
}

// D5 + D6 true: day overflow rolls the month (Jan 31 23:00 + 2 h -> Feb 1 01:00)
TEST(DateTime, DayRollsIntoNextMonth) {
    DateTime dt(2000, 1, 31, 23, 0, 0, 0);
    DateTime r = dt + 7200.0;  // +2 h
    EXPECT_EQ(r.month, 2u);
    EXPECT_EQ(r.day, 1u);
    EXPECT_EQ(r.hour, 1u);
}

// D6 true + D7: month overflow rolls the year
// (Dec 31 23:00 + 2 h -> next year Jan 1 01:00).
TEST(DateTime, MonthRollsIntoNextYear) {
    DateTime dt(2000, 12, 31, 23, 0, 0, 0);
    DateTime r = dt + 7200.0;
    EXPECT_EQ(r.year, 2001u);
    EXPECT_EQ(r.month, 1u);
    EXPECT_EQ(r.day, 1u);
    EXPECT_EQ(r.hour, 1u);
}

// D7 leap-year true branch: 2000 is divisible by 400 -> leap, Feb has 29 days.
TEST(DateTime, LeapYear2000HasFeb29) {
    DateTime dt(2000, 2, 28, 23, 0, 0, 0);
    DateTime r = dt + 7200.0;  // +2 h -> Feb 29 (leap), not Mar 1
    EXPECT_EQ(r.month, 2u);
    EXPECT_EQ(r.day, 29u);
}

// D7 leap-year false branch: 1900 divisible by 100 but not 400 -> NOT leap.
TEST(DateTime, NonLeapYear1900SkipsFeb29) {
    DateTime dt(1900, 2, 28, 23, 0, 0, 0);
    DateTime r = dt + 7200.0;  // +2 h -> Mar 1 (no Feb 29 in 1900)
    EXPECT_EQ(r.month, 3u);
    EXPECT_EQ(r.day, 1u);
}

// D7 ordinary leap year (div by 4, not by 100): 2024 -> Feb 29 exists.
TEST(DateTime, OrdinaryLeapYear2024HasFeb29) {
    DateTime dt(2024, 2, 28, 23, 0, 0, 0);
    DateTime r = dt + 7200.0;
    EXPECT_EQ(r.month, 2u);
    EXPECT_EQ(r.day, 29u);
}

// All decisions false: a small addition that triggers no carry at all.
TEST(DateTime, NoCarryWhenWellWithinBounds) {
    DateTime dt(2021, 6, 15, 10, 20, 30, 0);
    DateTime r = dt + 5.0;  // 30 + 5 = 35 s, no carries
    EXPECT_EQ(r.year, 2021u);
    EXPECT_EQ(r.month, 6u);
    EXPECT_EQ(r.day, 15u);
    EXPECT_EQ(r.hour, 10u);
    EXPECT_EQ(r.min, 20u);
    EXPECT_EQ(r.sec, 35u);
    EXPECT_EQ(r.msec, 0u);
}

// D5 loop iterating more than once: a large multi-day advance.
TEST(DateTime, MultiDayAdvanceAcrossMonthBoundary) {
    // Jan 30 12:00 + 5 days (432000 s) -> Feb 4 12:00 (2000 leap, Jan has 31).
    DateTime dt(2000, 1, 30, 12, 0, 0, 0);
    DateTime r = dt + 432000.0;
    EXPECT_EQ(r.month, 2u);
    EXPECT_EQ(r.day, 4u);
    EXPECT_EQ(r.hour, 12u);
}
