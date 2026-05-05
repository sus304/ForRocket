#!/bin/bash
set -e

FORROCKET_VER_DOT="4.1.12"
FORROCKET_VER_US="4_1_12"
CURRENT_DATE=$(date '+%Y%m%d%H%M%S')
BUILD_DIR=build-win
PACKAGE_DIR=ForRocket_v${FORROCKET_VER_DOT}_${CURRENT_DATE}

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "${SCRIPT_DIR}"

rm -rf "${BUILD_DIR}"

cmake -B "${BUILD_DIR}" \
    -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-x86_64.cmake \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" --parallel

mkdir "${PACKAGE_DIR}"

cp "${BUILD_DIR}/ForRocket.exe" "${PACKAGE_DIR}/"

cp examples/sample_CA.csv              "${PACKAGE_DIR}/"
cp examples/sample_config_area.json    "${PACKAGE_DIR}/"
cp examples/sample_param_engine.json   "${PACKAGE_DIR}/"
cp examples/sample_param_rocket.json   "${PACKAGE_DIR}/"
cp examples/sample_sequence_of_event.json "${PACKAGE_DIR}/"
cp examples/sample_config_solver.json  "${PACKAGE_DIR}/"
cp examples/sample_config_list_stage1.json "${PACKAGE_DIR}/"
cp examples/sample_thrust.csv          "${PACKAGE_DIR}/"
cp examples/sample_wind.csv            "${PACKAGE_DIR}/"

zip -r "ForRocket_v${FORROCKET_VER_US}_${CURRENT_DATE}.zip" "${PACKAGE_DIR}/"

rm -rf "${BUILD_DIR}"
rm -rf "${PACKAGE_DIR}/"
