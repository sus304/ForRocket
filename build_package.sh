#!/bin/bash

FORROCKET_VER_DOT="4.1.8"
FORROCKET_VER_US="4_1_8"
CURRENT_DATE=`date '+%Y%m%d%H%M%S'`
PACKAGE_DIR=ForRocket_v${FORROCKET_VER_DOT}_${CURRENT_DATE}

rm -rf build/

mkdir build
cd build
cmake .. -DCMAKE_CXX_FLAGS="-Wall" -DCMAKE_BUILD_TYPE=Release -G "MSYS Makefiles"
make
cd ..

mkdir ${PACKAGE_DIR}

cp build/ForRocket.exe ${PACKAGE_DIR}

cp examples/sample_CA.csv ${PACKAGE_DIR}
cp examples/sample_config_area.json ${PACKAGE_DIR}
cp examples/sample_param_engine.json ${PACKAGE_DIR}
cp examples/sample_param_rocket.json ${PACKAGE_DIR}
cp examples/sample_sequence_of_event.json ${PACKAGE_DIR}
cp examples/sample_config_solver.json ${PACKAGE_DIR}
cp examples/sample_config_list_stage1.json ${PACKAGE_DIR}
cp examples/sample_thrust.csv ${PACKAGE_DIR}
cp examples/sample_wind.csv ${PACKAGE_DIR}

zip -r ForRocket_v${FORROCKET_VER_US}_${CURRENT_DATE} ${PACKAGE_DIR}/

rm -rf build/
rm -rf ${PACKAGE_DIR}/
