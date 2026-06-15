#!/bin/bash
# GoogleTest is required only for the C++ unit tests under tests/unit/.
# It is intentionally NOT chained into setup_libs.sh so the production build
# stays dependency-light. Run this once before building with -DBUILD_TESTING=ON.
# release-1.12.1 is the last GoogleTest line that supports C++11 (the project
# standard); 1.13.0+ requires C++14.

GTEST_VER="1.12.1"

rm -rf googletest/

mkdir setup
cd setup

wget -O googletest-${GTEST_VER}.tar.gz https://github.com/google/googletest/archive/refs/tags/release-${GTEST_VER}.tar.gz
tar xzvf googletest-${GTEST_VER}.tar.gz

cd ../
cp -r -a setup/googletest-release-${GTEST_VER}/ googletest/
rm -rf setup/
