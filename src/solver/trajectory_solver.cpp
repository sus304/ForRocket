// ******************************************************
// Project Name    : ForRocket
// File Name       : trajectory_solver.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "trajectory_solver.hpp"

#include "Eigen/Core"

#include "json_control.hpp"

#include "factory/rocket_stage_factory.hpp"
#include "environment/datetime.hpp"
#include "rocket/rocket.hpp"
#include "dynamics/dynamics_base.hpp"

#ifdef DEBUG
#include <iostream>
#endif

forrocket::TrajectorySolver::TrajectorySolver(std::string solver_config_json_file) {
    JsonControl jc_solver_config(solver_config_json_file);
    
    model_id = jc_solver_config.getString("Model ID");
    number_stage = jc_solver_config.getInt("Number of Stage");

    // Making Stage instance
    RocketStageFactory stage_factory;
    for (int i=1; i <= number_stage; ++i) {
        auto jc_stage = JsonControl(jc_solver_config.getString("Stage"+std::to_string(i)+" Config File List"));
        stage_vector.push_back(stage_factory.Create(i, 
                                                jc_stage.getString("Rocket Configuration File Path"),
                                                jc_stage.getString("Engine Configuration File Path"),
                                                jc_stage.getString("Sequence of Event File Path")));
    }

    for (auto& stage : stage_vector) {
        stage.fdr = FlightDataRecorder(&stage.rocket);
    }

    // Prepare Launch - Master Clock
    DateTime launch_date(jc_solver_config.getString("Launch DateTime"));
    master_clock = SequenceClock(launch_date, 0.0);

    // Prepare Launch - Position Velocity Attitude
    Rocket& rocket_first_stage = stage_vector[0].rocket;
    auto jc_launch = jc_solver_config.getSubItem("Launch Condition");

    Eigen::Vector3d pos_init_LLH;
    pos_init_LLH << jc_launch.getDouble("Latitude [deg]"), jc_launch.getDouble("Longitude [deg]"), jc_launch.getDouble("Height for WGS84 [m]");

    rocket_first_stage.position.Initialize(master_clock.UTC_date_init, pos_init_LLH);

    Eigen::Vector3d vel_init_NED;
    vel_init_NED << jc_launch.getDouble("North Velocity [m/s]"), jc_launch.getDouble("East Velocity [m/s]"), jc_launch.getDouble("Down Velocity [m/s]");
    rocket_first_stage.velocity.Initialize(master_clock.UTC_date_init, vel_init_NED, pos_init_LLH, rocket_first_stage.position.ECI);

    Eigen::Vector3d euler;
    euler << jc_launch.getDouble("Azimuth [deg]"), jc_launch.getDouble("Elevation [deg]"), 0.0;
    euler = euler / 180.0 * 3.14159265;
    rocket_first_stage.attitude.Initialize(euler);

    rocket_first_stage.angular_velocity << 0.0, 0.0, 0.0;

    // Prepare Launch - Wind
    auto jc_wind = jc_solver_config.getSubItem("Wind Condition");
    if (jc_wind.getBool("Enable Wind")) {
        wind = EnvironmentWind(jc_wind.getString("Wind File Path"));
    } else {
        wind = EnvironmentWind(false);
    }
};


void forrocket::TrajectorySolver::Solve() {
    DynamicsBase::state x0;  // ECI pos, ECI vel, quat, angle vel, mass
    for (int i=0; i < stage_vector.size(); ++i) {
        RocketStage& stage = stage_vector[i];
        Rocket& rocket = stage.rocket;
        x0 = {rocket.position.ECI(0), rocket.position.ECI(1), rocket.position.ECI(2),
                rocket.velocity.ECI(0),rocket.velocity.ECI(1),rocket.velocity.ECI(2),
                rocket.attitude.quaternion(0),rocket.attitude.quaternion(1),rocket.attitude.quaternion(2),rocket.attitude.quaternion(3),
                rocket.angular_velocity(0),rocket.angular_velocity(1),rocket.angular_velocity(2),
                rocket.mass.propellant};
        stage.FlightSequence(&master_clock, &wind, x0);

        // Update next stage
        if (stage_vector.size() != i+1 && stage.enable_sepation) {
            RocketStage& next_stage = stage_vector[i+1];

            next_stage.time_start = stage.time_separation;

            next_stage.rocket.position.ECI(0) = x0[0];
            next_stage.rocket.position.ECI(1) = x0[1];
            next_stage.rocket.position.ECI(2) = x0[2];
            next_stage.rocket.velocity.ECI(0) = x0[3];
            next_stage.rocket.velocity.ECI(1) = x0[4];
            next_stage.rocket.velocity.ECI(2) = x0[5];
            next_stage.rocket.attitude.quaternion(0) = x0[6];
            next_stage.rocket.attitude.quaternion(1) = x0[7];
            next_stage.rocket.attitude.quaternion(2) = x0[8];
            next_stage.rocket.attitude.quaternion(3) = x0[9];
            next_stage.rocket.angular_velocity(0) = x0[10];
            next_stage.rocket.angular_velocity(1) = x0[11];
            next_stage.rocket.angular_velocity(2) = x0[12];
        }
    }
    
};

void forrocket::TrajectorySolver::DumpResult(bool minimum_dump) {
    for (int i=0, size=stage_vector.size(); i < size; ++i) {
        std::string name = model_id + "_stage" + std::to_string(stage_vector[i].stage_number) + "_flight_log";
        stage_vector[i].fdr.DumpCsv(name + ".csv", !minimum_dump);
    }
};


