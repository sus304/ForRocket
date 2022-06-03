#!/bin/bash

rm -rf build/

mkdir build
cd build
cmake .. -DCMAKE_CXX_FLAGS="-Wall" -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles"
make
cd ..