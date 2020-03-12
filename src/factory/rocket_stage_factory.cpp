// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_stage_factory.cpp
// Creation Date   : 2020/02/06
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket_stage_factory.hpp"

#include "json_control.hpp"
#include "factory/rocket_factory.hpp"

forrocket::RocketStage forrocket::RocketStageFactory::Create(const int stage_number,
                                                                std::string rocket_config_json_file, 
                                                                std::string engine_config_json_file,
                                                                std::string sequence_of_event_json_file) {
    
    RocketFactory rocket_factory;
    RocketStage rocket_stage(stage_number, rocket_factory.Create(rocket_config_json_file, engine_config_json_file));

    JsonControl jc(sequence_of_event_json_file);

    rocket_stage.time_start = jc.getDouble("Flight Start Time [s]");
    rocket_stage.time_ignittion = jc.getDouble("Engine Ignittion Time [s]");

    rocket_stage.enable_launcher = jc.getBool("Enable Rail-Launcher Launch");
    if (rocket_stage.enable_launcher) {
        rocket_stage.length_launcher_rail = jc.getSubItem("Rail Launcher").getDouble("Length [m]");
    }
    
    rocket_stage.enable_cutoff = jc.getBool("Enable Engine Cutoff");
    if (rocket_stage.enable_cutoff) {
        rocket_stage.time_cutoff = jc.getSubItem("Cutoff").getDouble("Cutoff Time [s]");
        rocket_stage.rocket.engine.burn_duration = rocket_stage.time_cutoff;
    }
    
    rocket_stage.enable_program_attitude = jc.getBool("Enable Program Attitude");
    if (rocket_stage.enable_program_attitude) {
        auto jc_attitude_control = jc.getSubItem("Attitude Control");
        rocket_stage.time_start_attitude_controll = jc_attitude_control.getDouble("Start Time [s]");
        rocket_stage.time_end_attitude_controll = jc_attitude_control.getDouble("End Time [s]");
    }

    rocket_stage.enable_sepation = jc.getBool("Enable Stage Separation");
    if (rocket_stage.enable_sepation) {
        auto jc_separation = jc.getSubItem("Upper Stage");
        rocket_stage.time_separation = jc_separation.getDouble("Stage Separation Time [s]");
        rocket_stage.mass_upper_stage = jc_separation.getDouble("Upper Stage Mass [kg]");
    }

    rocket_stage.enable_despin = jc.getBool("Enable Despin Control");
    if (rocket_stage.enable_despin) {
        rocket_stage.time_despin = jc.getSubItem("Despin").getDouble("Time [s]");
    }

    rocket_stage.enable_fairing_jettson = jc.getBool("Enable Fairing Jettson");
    if (rocket_stage.enable_fairing_jettson) {
        auto jc_fairing_jettson = jc.getSubItem("Fairing");
        rocket_stage.time_jettson_fairing = jc_fairing_jettson.getDouble("Jettson Time [s]");
        rocket_stage.mass_fairing = jc_fairing_jettson.getDouble("Mass [s]");
    }

    rocket_stage.enable_parachute_open = jc.getBool("Enable Parachute Open");
    if (rocket_stage.enable_parachute_open) {
        auto jc_parachute = jc.getSubItem("Parachute");
        rocket_stage.enable_apogee_parachute_open = jc_parachute.getDouble("Enable Forced Apogee Open");
        rocket_stage.time_open_parachute = jc_parachute.getDouble("Open Time [s]");
    }

    rocket_stage.exist_second_parachute = jc.getBool("Enable Secondary Parachute Open");
    if (rocket_stage.exist_second_parachute) {
        auto jc_secondary_parachute = jc.getSubItem("Secondary Parachute");
        rocket_stage.time_open_second_parachute = jc_secondary_parachute.getDouble("Open Time [s]");
    }
    
    if (jc.getBool("Enable Auto Terminate SubOrbital Flight")) {
        double safety_factor = 1.4;
        rocket_stage.time_end = safety_factor * std::abs(60.0 * std::log(rocket_stage.rocket.engine.total_impulse) - 410.0);        
    } else {
        rocket_stage.time_end = jc.getDouble("Flight End Time [s]");
    }
    // rocket_stage.time_step = std::pow(10.0, std::floor((int)std::log10((int)rocket_stage.time_end)+1) - 5);
    rocket_stage.time_step = jc.getDouble("Time Step [s]");
    
    return rocket_stage;
}



