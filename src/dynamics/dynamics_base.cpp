// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_base.cpp
// Creation Date   : 2019/10/20

// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "dynamics_base.hpp"

#include <cmath>

#include "degrad.hpp"


Eigen::Vector3d forrocket::DynamicsBase::AeroForce(Rocket* p_rocket) {
    Eigen::Vector3d force_aero;

    double force_axial = p_rocket->dynamic_pressure * p_rocket->CA * p_rocket->area;
    double force_normal = p_rocket->dynamic_pressure * p_rocket->CNa * p_rocket->area;
    double force_normal_y_axis = force_normal * p_rocket->sideslip_angle;
    double force_normal_z_axis = force_normal * p_rocket->angle_of_attack;
    force_aero << -force_axial, force_normal_y_axis, -force_normal_z_axis;

    return force_aero;
};


Eigen::Vector3d forrocket::DynamicsBase::GyroEffectMoment(Rocket* p_rocket) {
    Eigen::Vector3d gyro_effect;

    Eigen::Vector3d angular_momentum = p_rocket->getInertiaTensor() * p_rocket->angular_velocity;
    gyro_effect = p_rocket->angular_velocity.cross(angular_momentum);

    return -1.0 * gyro_effect;
};


Eigen::Vector3d forrocket::DynamicsBase::ThrustMoment(Rocket* p_rocket) {
    Eigen::Vector3d moment_thrust;

    Eigen::Vector3d moment_arm(p_rocket->length_CG - p_rocket->length_thrust,
                                p_rocket->y_CG - p_rocket->y_thrust_offset,
                                p_rocket->z_CG - p_rocket->z_thrust_offset);
    moment_thrust = p_rocket->force.thrust.cross(moment_arm);

    return moment_thrust;
};


Eigen::Vector3d forrocket::DynamicsBase::AeroForceMoment(Rocket* p_rocket) {
    Eigen::Vector3d moment_aero;

    Eigen::Vector3d moment_arm(p_rocket->length_CG - p_rocket->length_CP, 0.0, 0.0);
    moment_aero = p_rocket->force.aero.cross(moment_arm);
    moment_aero[0] = p_rocket->dynamic_pressure * p_rocket->Cld * p_rocket->area * p_rocket->diameter * p_rocket->cant_angle_fin;

    return moment_aero;
};


Eigen::Vector3d forrocket::DynamicsBase::AeroDampingMoment(Rocket* p_rocket) {
    Eigen::Vector3d moment_aero_dumping;

    const double airspeed = p_rocket->velocity.air_body.norm();
    if (airspeed <= 0.0) {
        moment_aero_dumping << 0.0, 0.0, 0.0;
        return moment_aero_dumping;
    }

    Eigen::Vector3d coefficient_aero_dumping(p_rocket->Clp, p_rocket->Cmq, p_rocket->Cnr);
    moment_aero_dumping = p_rocket->dynamic_pressure * coefficient_aero_dumping.array() * p_rocket->area * std::pow(p_rocket->diameter, 2)
                            / (2.0 * airspeed) * p_rocket->angular_velocity.array();

    return moment_aero_dumping;
};


Eigen::Vector3d forrocket::DynamicsBase::JetDampingMoment(Rocket* p_rocket) {
    Eigen::Vector3d moment_jet_dumping;

    // ジェットダンピング: 排気がノズル出口で持ち去る角運動量による減衰モーメント M = -mdot * l^2 * omega
    //   pitch/yaw (横軸): l = CG〜ノズル出口の軸距離。機体後端≒ノズル出口と仮定し length_CG を使用
    //   roll (機軸):      l^2 = 排気の機軸まわり慣動半径^2 = r_e^2 / 2 = A_exit / (2*pi) (中央単ノズル一様分布)
    const double mdot = p_rocket->engine.mdot_prop;
    const double arm_transverse = p_rocket->length_CG;
    const double k_roll_sq = p_rocket->engine.getAreaExit() / (2.0 * pi);

    moment_jet_dumping[0] = -mdot * k_roll_sq * p_rocket->angular_velocity[0];
    moment_jet_dumping[1] = -mdot * arm_transverse * arm_transverse * p_rocket->angular_velocity[1];
    moment_jet_dumping[2] = -mdot * arm_transverse * arm_transverse * p_rocket->angular_velocity[2];

    return moment_jet_dumping;
};


Eigen::Vector3d forrocket::DynamicsBase::GasJetMoment(Rocket* p_rocket, double t) {
    Eigen::Vector3d moment = Eigen::Vector3d::Zero();
    if (!p_rocket->gas_jet_config.enable) return moment;
    double elapsed = t - p_rocket->time_launch_clear;
    if (elapsed >= 0.0 && elapsed <= p_rocket->gas_jet_config.duration) {
        moment[0] = p_rocket->gas_jet_config.rolling_moment;
    }
    return moment;
};


Eigen::Matrix4d forrocket::DynamicsBase::QuaternionDiff(Rocket* p_rocket) {
    double p = p_rocket->angular_velocity[0];
    double q = p_rocket->angular_velocity[1];
    double r = p_rocket->angular_velocity[2];
    Eigen::Matrix4d quat_dot;
    quat_dot << 0, r, -q, p,
                -r, 0, p, q,
                q, -p, 0, r,
                -p, -q, -r, 0;
    return quat_dot;
};

