// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_stage_factory.cpp
// Creation Date   : 2020/02/06
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket_stage_factory.hpp"

#include "factory/rocket_factory.hpp"

forrocket::RocketStage forrocket::RocketStageFactory::Create(const int stage_number,
                                                                                    std::string rocket_config_json_file, 
                                                                                    std::string engine_config_json_file,
                                                                                    std::string sequence_of_event_json_file) {
    
    RocketFactory rocket_factory;
    RocketStage rocket_stage(stage_number, rocket_factory.Create(rocket_config_json_file, engine_config_json_file));

    rocket_stage.time_start = 0.0;
    rocket_stage.time_ignittion = 0.0;
    rocket_stage.enable_launcher = true;
    rocket_stage.length_launcher_rail = 5.0;
    rocket_stage.enable_cutoff = false;
    rocket_stage.time_cutoff = 2.0;
    rocket_stage.enable_program_attitude = false;
    // rocket_stage.time_start_attitude_controll = ;
    // rocket_stage.time_end_attitude_controll = ;
    rocket_stage.enable_sepation = false;
    // rocket_stage.time_separation = ;
    // rocket_stage.mass_upper_stage = ;
    rocket_stage.enable_despin = false;
    // rocket_stage.time_despin = ;
    rocket_stage.enable_fairing_jettson = false;
    // rocket_stage.time_jettson_fairing = ;
    // rocket_stage.mass_fairing = ;
    rocket_stage.enable_parachute_open = false;
    // rocket_stage.enable_apogee_parachute_open = ;
    // rocket_stage.time_open_parachute = ;
    rocket_stage.exist_second_parachute = false;
    // rocket_stage.time_open_second_parachute = ;
    rocket_stage.time_end = 300.0;

    return rocket_stage;
}



