cmake_minimum_required(VERSION 3.13)

project(ForRocket CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (WIN32)
	set(CMAKE_CXX_FLAGS "-Wall -static -lstdc++ -lgcc -lwinpthread")
else()
	set(CMAKE_CXX_FLAGS "-Wall -lpthread")
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DDEBUG")
# -O3 is equal-or-slightly-faster than -O2 for this workload (small fixed-size
# Eigen ops + adaptive ODE loop) and matches the shipped v4.3.x binaries.
# -march=native was measured SLOWER here (AVX overhead on tiny matrices), so it
# is intentionally not enabled.
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

add_definitions(-DEIGEN_MPL2_ONLY)

include_directories(lib)
include_directories(inc)
include_directories(inc/dynamics)
include_directories(inc/environment)
include_directories(inc/factory)
include_directories(inc/rocket)
include_directories(inc/rocket/parameter)
include_directories(inc/solver)

set(srcs
	src/ForRocket.cpp
	src/degrad.cpp
	src/commandline_option.cpp
	src/fileio.cpp
	src/json_control.cpp
	src/interpolate.cpp
	src/environment/coordinate.cpp
	src/environment/datetime.cpp
	src/environment/sequence_clock.cpp
	src/environment/satmo1976.cpp
	src/environment/air.cpp
	src/environment/gravity.cpp
	src/environment/wgs84.cpp
	src/environment/wind.cpp
	src/environment/vincenty.cpp
	src/rocket/parameter/acceleration.cpp
	src/rocket/parameter/attitude.cpp
	src/rocket/parameter/force.cpp
	src/rocket/parameter/interpolate_parameter.cpp
	src/rocket/parameter/mass.cpp
	src/rocket/parameter/moment.cpp
	src/rocket/parameter/position.cpp
	src/rocket/parameter/velocity.cpp
	src/rocket/engine.cpp
	src/rocket/flight_data_recorder.cpp
	src/rocket/rocket.cpp
	src/dynamics/dynamics_base.cpp
	src/dynamics/flight_dynamics.cpp
	src/factory/engine_factory.cpp
	src/factory/rocket_factory.cpp
	src/factory/rocket_stage_factory.cpp
	src/solver/flight_event.cpp
	src/solver/rocket_stage.cpp
	src/solver/trajectory_solver.cpp
)

# Build the simulation modules as a library so the executable, the unit tests,
# and coverage runs all share one compilation of the source.
set(core_srcs ${srcs})
list(REMOVE_ITEM core_srcs src/ForRocket.cpp)
add_library(forrocket_core STATIC ${core_srcs})

add_executable(ForRocket src/ForRocket.cpp)
target_link_libraries(ForRocket forrocket_core)

# --- Unit tests & coverage (opt-in; production build is unaffected) -------
option(BUILD_TESTING "Build the C++ unit tests (GoogleTest)" OFF)
option(ENABLE_COVERAGE "Instrument forrocket_core for gcov/gcovr coverage" OFF)

if(ENABLE_COVERAGE)
	# PUBLIC so the flags propagate to whatever links forrocket_core (the test
	# binary), which is what gcov needs to emit .gcda/.gcno for the core sources.
	target_compile_options(forrocket_core PUBLIC -O0 -g --coverage)
	target_link_options(forrocket_core PUBLIC --coverage)
endif()

if(BUILD_TESTING)
	enable_testing()
	add_subdirectory(tests/unit)
endif()

