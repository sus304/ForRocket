# C++ unit tests (GoogleTest). Enabled from the top-level build with
# -DBUILD_TESTING=ON. See tests/unit/README.md.

# --- Acquire GoogleTest ---------------------------------------------------
# Prefer the vendored copy (lib/setup_googletest.sh) so builds stay offline,
# matching how Eigen/Boost/json are handled. Fall back to FetchContent when it
# is absent (requires network).
set(GTEST_VENDORED_DIR ${CMAKE_SOURCE_DIR}/lib/googletest)
if(EXISTS ${GTEST_VENDORED_DIR}/CMakeLists.txt)
    message(STATUS "ForRocket tests: using vendored GoogleTest at ${GTEST_VENDORED_DIR}")
    add_subdirectory(${GTEST_VENDORED_DIR} ${CMAKE_BINARY_DIR}/_googletest EXCLUDE_FROM_ALL)
else()
    message(STATUS "ForRocket tests: vendored GoogleTest not found; fetching release-1.12.1 (network required). Run lib/setup_googletest.sh for offline builds.")
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.tar.gz
    )
    FetchContent_MakeAvailable(googletest)
endif()

# --- Test executable ------------------------------------------------------
add_executable(forrocket_unittests
    # utility / math
    test_degrad.cpp
    test_interpolate.cpp
    # environment
    test_wgs84.cpp
    test_coordinate.cpp
    test_gravity.cpp
    test_air.cpp
    test_satmo1976.cpp
    test_vincenty.cpp
    test_datetime.cpp
    test_wind.cpp
    test_sequence_clock.cpp
    # rocket parameters
    test_param_mass.cpp
    test_param_acceleration.cpp
    test_param_force.cpp
    test_param_moment.cpp
    test_param_interpolate.cpp
    test_param_attitude.cpp
    test_param_position.cpp
    test_param_velocity.cpp
    # rocket / engine (Tier B — uses test_fixtures.hpp)
    test_engine.cpp
    test_rocket.cpp
    test_flight_data_recorder.cpp
    # dynamics / solver (Tier B — uses test_fixtures.hpp)
    test_dynamics_base.cpp
    test_flight_dynamics.cpp
    test_flight_dynamics_program.cpp
    test_rocket_stage.cpp
    test_flight_event.cpp
    # Tier C — JSON control, factories (error paths), solver smoke
    test_json_control.cpp
    test_engine_factory.cpp
    test_rocket_factory.cpp
    test_rocket_stage_factory.cpp
    test_trajectory_solver.cpp
    # test_noniterative_iip.cpp  # disabled: src/dynamics/noniterative_iip.cpp
    #   is not in the production build (only caller is commented out, see
    #   flight_data_recorder.cpp:442). Dead code — revive or delete first.
)
target_link_libraries(forrocket_unittests PRIVATE forrocket_core GTest::gtest_main)

# Register the suite as a single CTest test. (gtest_discover_tests is avoided:
# on CMake 3.16 + single-config Makefiles its per-test include file is not
# picked up by `ctest`.) GoogleTest's own console output still reports each
# TEST() individually; for CI, add --gtest_output=xml:results.xml for JUnit.
add_test(NAME forrocket_unittests COMMAND forrocket_unittests)
