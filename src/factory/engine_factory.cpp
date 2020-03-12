// ******************************************************
// Project Name    : ForRocket
// File Name       : engine_factory.cpp
// Creation Date   : 2019/11/24
//
// Copyright (c) 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "engine_factory.hpp"

#include <cmath>

#include "degrad.hpp"
#include "fileio.hpp"
#include "json_control.hpp"


forrocket::Engine forrocket::EngineFactory::Create(std::string engine_config_json_file) {
    JsonControl jc(engine_config_json_file);

    double diameter_exit = jc.getDouble("Nozzle Exit Diameter [mm]") / 1e3;
    double area_exit = 0.25 * std::pow(diameter_exit, 2) * pi;

    bool enable_mis_alignment = jc.getBool("Enable Engine Miss Alignment");
    if (jc.getBool("Enable Thrust File")) {
        JsonControl jc_file = jc.getSubItem("Thrust File");
        auto thrust_log = LoadCsvLog(jc_file.getString("Thrust at vacuum File Path"));
        if (enable_mis_alignment) {
            double mis_alignment_y = jc.getSubItem("Engine Miss-Alignment").getDouble("y-Axis Angle [deg]") / 180.0 * 3.14159265;
            double mis_alignment_z = jc.getSubItem("Engine Miss-Alignment").getDouble("z-Axis Angle [deg]") / 180.0 * 3.14159265;
            return Engine(thrust_log[0], thrust_log[1], thrust_log[2], area_exit, mis_alignment_y, mis_alignment_z);
        } else {
            return Engine(thrust_log[0], thrust_log[1], thrust_log[2], area_exit);
        }
    } else {
        JsonControl jc_const = jc.getSubItem("Constant Thrust");
        double thrust = jc_const.getDouble("Thrust at vacuum [N]");
        double mdot_p = jc_const.getDouble("Propellant Mass Flow Rate [kg/s]");
        double burn_duration = jc_const.getDouble("Burn Duration [sec]");
        if (enable_mis_alignment) {
            double mis_alignment_y = jc.getSubItem("Engine Miss-Alignment").getDouble("y-Axis Angle [deg]") / 180.0 * 3.14159265;
            double mis_alignment_z = jc.getSubItem("Engine Miss-Alignment").getDouble("z-Axis Angle [deg]") / 180.0 * 3.14159265;
            return Engine(burn_duration, thrust, mdot_p, area_exit, mis_alignment_y, mis_alignment_z);
        } else {
            return Engine(burn_duration, thrust, mdot_p, area_exit);
        }
    }
};

