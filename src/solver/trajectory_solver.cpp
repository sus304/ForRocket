// ******************************************************
// Project Name    : ForRocket
// File Name       : trajectory_solver.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "trajectory_solver.hpp"

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"

#include "json_control.hpp"

#include "factory/rocket_stage_factory.hpp"
#include "environment/datetime.hpp"
#include "rocket/rocket.hpp"
#include "dynamics/dynamics_base.hpp"

forrocket::TrajectorySolver::TrajectorySolver(std::string launch_config_json_file, std::string stage_list_json_file) {
    JsonControl stage_list_json(stage_list_json_file);

    // number_stage = stage_list_json.getInt("number of stage");
    number_stage = 1;

    RocketStageFactory stage_factory;
    stage_vector.push_back(stage_factory.Create(1, "r.json", "e.json", "soe.json"));

    for (auto& stage : stage_vector) {
        stage.fdr = FlightDataRecorder(&stage.rocket);
    }

    // Prepare Launch - Master Clock
    DateTime launch_date;
    master_clock = SequenceClock(launch_date, 0.0);

    // Prepare Launch - Position Velocity Attitude
    Rocket& rocket_first_stage = stage_vector[0].rocket;
    JsonControl launch_config_json(launch_config_json_file);
    
    Eigen::Vector3d pllh;
    pllh << 35.2, 135.3, 10.0;
    rocket_first_stage.position.Initialize(master_clock.UTC_date_init, pllh);
    #ifdef DEBUG_NO
        std::cout << rocket_first_stage.position.LLH << std::endl;
        std::cout << rocket_first_stage.position.ECEF << std::endl;
    #endif

    Eigen::Vector3d vned;
    vned << 0.0, 0.0, 0.0;
    rocket_first_stage.velocity.Initialize(master_clock.UTC_date_init, vned, pllh, rocket_first_stage.position.ECI);
    #ifdef DEBUG_NO
        std::cout << rocket_first_stage.velocity.NED << std::endl;
        std::cout << rocket_first_stage.velocity.ECEF << std::endl;
    #endif

    double elv_init = 85.0 / 180.0 * 3.14159265;
    double azi_init = 275.0 / 180.0 * 3.14159265;
    double roll_init = 0.0 / 180.0 * 3.14159265;
    Eigen::Vector3d euler;
    euler << azi_init, elv_init, roll_init;
    rocket_first_stage.attitude.Initialize(euler);
    rocket_first_stage.angular_velocity << 0.0, 0.0, 0.0;

    // Prepare Launch - Wind
    wind = EnvironmentWind();
};


void forrocket::TrajectorySolver::Solve() {
    #ifdef DEBUG
        std::cout << "Trajectory Solver Start" << std::endl;
    #endif

    // ECI pos, ECI vel, quat, angle vel, mass
    DynamicsBase::state x0;
    for (int i=0; i < stage_vector.size(); ++i) {
        RocketStage& stage = stage_vector[i];
        Rocket& rocket = stage.rocket;
        x0 = {rocket.position.ECI(0), rocket.position.ECI(1), rocket.position.ECI(2),
                rocket.velocity.ECI(0),rocket.velocity.ECI(1),rocket.velocity.ECI(2),
                rocket.attitude.quaternion(0),rocket.attitude.quaternion(1),rocket.attitude.quaternion(2),rocket.attitude.quaternion(3),
                rocket.angular_velocity(0),rocket.angular_velocity(1),rocket.angular_velocity(2),
                rocket.mass.propellant};
        stage_vector[i].FlightSequence(&master_clock, &wind, x0);
    }
    
    #ifdef DEBUG
        std::cout << "Trajectory Solver End" << std::endl;
    #endif

};

