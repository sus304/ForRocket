cmake_minimum_required(VERSION 3.14)

project(ForRocket CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-Wall -static -lstdc++ -lgcc -lwinpthread")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

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
	src/dynamics/dynamics_3dof_onlauncher.cpp
	src/dynamics/dynamics_3dof_parachute.cpp
	src/dynamics/dynamics_6dof_aero.cpp
	src/dynamics/dynamics_6dof_programrate.cpp
	src/dynamics/noniterative_iip.cpp
	src/factory/engine_factory.cpp
	src/factory/rocket_factory.cpp
	src/factory/rocket_stage_factory.cpp
	src/solver/rocket_stage.cpp
	src/solver/trajectory_solver.cpp
)

add_executable(ForRocket ${srcs})

