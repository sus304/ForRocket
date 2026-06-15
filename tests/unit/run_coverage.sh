#!/bin/bash
# Configure + build the unit tests with coverage, run them, and emit a gcovr
# report. Run from anywhere; paths are resolved relative to the repo root.
#
# Requirements: cmake, g++, and gcovr (pip install gcovr).
# GoogleTest: run lib/setup_googletest.sh once for an offline build, otherwise
# the configure step fetches it over the network.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build_coverage"
REPORT_DIR="${BUILD_DIR}/coverage_report"

cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DENABLE_COVERAGE=ON
cmake --build "${BUILD_DIR}" --target forrocket_unittests -j"$(nproc)"

# Run the tests. (cd into the build dir rather than `ctest --test-dir`, which
# requires CMake >= 3.20; the project targets 3.13+.)
( cd "${BUILD_DIR}" && ctest --output-on-failure )

mkdir -p "${REPORT_DIR}"
# Coverage of the simulation sources only (filter to src/, drop tests + lib).
# Metrics we track: C0 = lines, C1 = branches. --exclude-throw-branches /
# --exclude-unreachable-branches keep the branch metric on real ForRocket
# decision points instead of compiler-generated exception edges + Eigen-inlined
# branches.
#
# We deliberately do NOT use gcovr --decisions: its experimental analyzer
# mis-reports consecutive single-line `if (cond) stmt;` blocks as uncovered even
# when gcov confirms both branches execute (it scored flight_dynamics.cpp 63%
# while true branch coverage was 100%). Branch coverage is the reliable C1 proxy.
gcovr --root "${REPO_ROOT}" \
    --filter "${REPO_ROOT}/src/" \
    --exclude-throw-branches \
    --exclude-unreachable-branches \
    --print-summary \
    --html-details "${REPORT_DIR}/index.html" \
    --xml "${BUILD_DIR}/coverage.xml" \
    "${BUILD_DIR}"

echo ""
echo "HTML report: ${REPORT_DIR}/index.html"
echo "Cobertura XML (for CI): ${BUILD_DIR}/coverage.xml"
