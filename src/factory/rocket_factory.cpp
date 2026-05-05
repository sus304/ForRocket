// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_factory.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket_factory.hpp"

#include <cmath>

#include "degrad.hpp"
#include "fileio.hpp"
#include "json_control.hpp"
#include "factory/engine_factory.hpp"
#include "rocket/parameter/interpolate_parameter.hpp"

forrocket::Rocket forrocket::RocketFactory::Create(std::string rocket_config_json_file, std::string engine_config_json_file) {
    Rocket rocket;

    EngineFactory engine_factory;
    rocket.engine = engine_factory.Create(engine_config_json_file);

    JsonControl jc(rocket_config_json_file);

    rocket.diameter = jc.getDouble("Diameter [mm]") / 1e3;
    rocket.area = 0.25 * std::pow(rocket.diameter, 2) * pi;
    rocket.length = jc.getDouble("Length [mm]") / 1e3;
    rocket.mass.inert = jc.getSubItem("Mass").getDouble("Inert [kg]");
    rocket.mass.propellant = jc.getSubItem("Mass").getDouble("Propellant [kg]");
    
    rocket.gas_jet_config.enable = jc.getBool("Enable Gas Jet");
    if (rocket.gas_jet_config.enable) {
        auto jc_gj = jc.getSubItem("Gas Jet");
        rocket.gas_jet_config.rolling_moment = jc_gj.getDouble("Rolling Moment [N.m]");
        rocket.gas_jet_config.duration = jc_gj.getDouble("Duration [s]");
    }

    rocket.enable_program_attitude = jc.getBool("Enable Program Attitude");
    if (rocket.enable_program_attitude) {
        auto jc_att = jc.getSubItem("Program Attitude");
        std::string mode = jc_att.getString("Mode");
        rocket.attitude_program_config.mode_rate   = (mode == "Rate");
        rocket.attitude_program_config.enable_yaw   = jc_att.getBool("Enable Yaw");
        rocket.attitude_program_config.enable_pitch = jc_att.getBool("Enable Pitch");
        rocket.attitude_program_config.enable_roll  = jc_att.getBool("Enable Roll");

        auto prog = LoadCsvLog(jc_att.getString("File Path"));
        // CSV values are in degrees (angle) or deg/s (rate); convert to rad or rad/s
        for (auto& v : prog[1]) v *= pi / 180.0;
        for (auto& v : prog[2]) v *= pi / 180.0;
        for (auto& v : prog[3]) v *= pi / 180.0;

        if (rocket.attitude_program_config.mode_rate) {
            rocket.setAttitudeProgramRate(
                InterpolateParameter(prog[0], prog[1], "same"),
                InterpolateParameter(prog[0], prog[2], "same"),
                InterpolateParameter(prog[0], prog[3], "same"));
        } else {
            rocket.setAttitudeProgram(
                InterpolateParameter(prog[0], prog[1], "same"),
                InterpolateParameter(prog[0], prog[2], "same"),
                InterpolateParameter(prog[0], prog[3], "same"));
        }
        rocket.time_start_attitude_control = prog[0].front();
        rocket.time_end_attitude_control = prog[0].back();
    }

    if (jc.getBool("Enable X-C.G. File")) {
        auto Xcg_log = LoadCsvLog(jc.getSubItem("X-C.G. File").getString("X-C.G. File Path"));
        rocket.setLengthCG(InterpolateParameter(Xcg_log[0], Xcg_log[1], "same"));
    } else {
        rocket.setLengthCG(InterpolateParameter(jc.getSubItem("Constant X-C.G.").getDouble("Constant X-C.G. from BodyTail [mm]") / 1e3));
    }

    if (jc.getBool("Enable M.I. File")) {
        auto MOI_log = LoadCsvLog(jc.getSubItem("M.I. File").getString("M.I. File Path"));
        auto yaw = InterpolateParameter(MOI_log[0], MOI_log[1], "same");
        auto pitch = InterpolateParameter(MOI_log[0], MOI_log[2], "same");
        auto roll = InterpolateParameter(MOI_log[0], MOI_log[3], "same");
        rocket.setInertiaTensor(roll, pitch, yaw);
    } else {
        auto jc_const = jc.getSubItem("Constant M.I.");
        auto yaw = InterpolateParameter(jc_const.getDouble("Yaw Axis [kg-m2]"));
        auto pitch = InterpolateParameter(jc_const.getDouble("Pitch Axis [kg-m2]"));
        auto roll = InterpolateParameter(jc_const.getDouble("Roll Axis [kg-m2]"));
        rocket.setInertiaTensor(roll, pitch, yaw);
    }

    if (jc.getBool("Enable X-C.P. File")) {
        auto Xcp_log = LoadCsvLog(jc.getSubItem("X-C.P. File").getString("X-C.P. File Path"));
        rocket.setLengthCP(InterpolateParameter(Xcp_log[0], Xcp_log[1], "same"));
    } else {
        rocket.setLengthCP(InterpolateParameter(jc.getSubItem("Constant X-C.P.").getDouble("Constant X-C.P. from BodyTail [mm]") / 1e3));
    }
  
    rocket.length_thrust = jc.getDouble("X-ThrustLoadingPoint from BodyTail [mm]") / 1e3;

    if (jc.getBool("Enable CA File")) {
        auto CA_log = LoadCsvLog(jc.getSubItem("CA File").getString("CA File Path"));
        auto CA_burnout_log = LoadCsvLog(jc.getSubItem("CA File").getString("BurnOut CA File Path"));
        rocket.setCA(InterpolateParameter(CA_log[0], CA_log[1], "same"), InterpolateParameter(CA_burnout_log[0], CA_burnout_log[1], "same"));
    } else {
        rocket.setCA(InterpolateParameter(jc.getSubItem("Constant CA").getDouble("Constant CA [-]")), 
                        InterpolateParameter(jc.getSubItem("Constant CA").getDouble("Constant BurnOut CA [-]")));
    }

    if (jc.getBool("Enable CNa File")) {
        auto CNa_log = LoadCsvLog(jc.getSubItem("CNa File").getString("CNa File Path"));
        rocket.setCNa(InterpolateParameter(CNa_log[0], CNa_log[1], "same"));
    } else {
        rocket.setCNa(InterpolateParameter(jc.getSubItem("Constant CNa").getDouble("Constant CNa [1/rad]"))); 
    }

    rocket.cant_angle_fin = jc.getDouble("Fin Cant Angle [deg]") / 180.0 * pi;
    if (jc.getBool("Enable Cld File")) {
        auto Cld_log = LoadCsvLog(jc.getSubItem("Cld File").getString("Cld File Path"));
        rocket.setCld(InterpolateParameter(Cld_log[0], Cld_log[1], "same"));
    } else {
        rocket.setCld(InterpolateParameter(jc.getSubItem("Constant Cld").getDouble("Constant Cld [1/rad]"))); 
    }

    if (jc.getBool("Enable Clp File")) {
        auto Clp_log = LoadCsvLog(jc.getSubItem("Clp File").getString("Clp File Path"));
        rocket.setClp(InterpolateParameter(Clp_log[0], Clp_log[1], "same"));
    } else {
        rocket.setClp(InterpolateParameter(jc.getSubItem("Constant Clp").getDouble("Constant Clp [-]"))); 
    }

    if (jc.getBool("Enable Cmq File")) {
        auto Cmq_log = LoadCsvLog(jc.getSubItem("Cmq File").getString("Cmq File Path"));
        rocket.setCmq(InterpolateParameter(Cmq_log[0], Cmq_log[1], "same"));
    } else {
        rocket.setCmq(InterpolateParameter(jc.getSubItem("Constant Cmq").getDouble("Constant Cmq [-]"))); 
    }

    if (jc.getBool("Enable Cnr File")) {
        auto Cnr_log = LoadCsvLog(jc.getSubItem("Cnr File").getString("Cnr File Path"));
        rocket.setCnr(InterpolateParameter(Cnr_log[0], Cnr_log[1], "same"));
    } else {
        rocket.setCnr(InterpolateParameter(jc.getSubItem("Constant Cnr").getDouble("Constant Cnr [-]"))); 
    }


    return rocket;
};





