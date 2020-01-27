// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_base.cpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "dynamics_base.hpp"

#include <cmath>


Eigen::Vector3d forrocket::DynamicsBase::AeroForce() {
    Eigen::Vector3d force_aero;

    double force_axial = p_rocket->dynamic_pressure * p_rocket->getCA(p_rocket->velocity.mach_number) * p_rocket->area;
    double force_normal = p_rocket->dynamic_pressure * p_rocket->getCNa(p_rocket->velocity.mach_number) * p_rocket->area;
    double force_normal_y_axis = force_normal * p_rocket->sideslip_angle;
    double force_normal_z_axis = force_normal * p_rocket->angle_of_attack;
    force_aero << force_axial, force_normal_y_axis, force_normal_z_axis;

    return force_aero;
};


Eigen::Vector3d forrocket::DynamicsBase::GyroEffectMoment() {
    Eigen::Vector3d gyro_effect;

    Eigen::Vector3d angular_momentum = p_rocket->getInertiaTensor() * p_rocket->angular_velocity;
    gyro_effect = p_rocket->angular_velocity.cross(angular_momentum);

    return gyro_effect;
};


Eigen::Vector3d forrocket::DynamicsBase::ThrustMoment() {
    Eigen::Vector3d moment_thrust;

    Eigen::Vector3d moment_arm(p_rocket->getLengthCG() - p_rocket->length_thrust, 0.0, 0.0);
    moment_thrust = p_rocket->force.thrust.cross(moment_arm);
    
    return moment_thrust;
};


Eigen::Vector3d forrocket::DynamicsBase::AeroForceMoment() {
    Eigen::Vector3d moment_aero;

    Eigen::Vector3d moment_arm(p_rocket->getLengthCG() - p_rocket->getLengthCP(p_rocket->velocity.mach_number), 0.0, 0.0);
    moment_aero = p_rocket->force.aero.cross(moment_arm);
    moment_aero[0] += p_rocket->dynamic_pressure * p_rocket->getCld(p_rocket->velocity.mach_number) * p_rocket->area * p_rocket->cant_angle_fin * 4;

    return moment_aero;
};


Eigen::Vector3d forrocket::DynamicsBase::AeroDampingMoment() {
    Eigen::Vector3d moment_aero_dumping;

    Eigen::Vector3d coefficient_aero_dumping(p_rocket->getClp(p_rocket->velocity.mach_number),
                                                p_rocket->getCmq(p_rocket->velocity.mach_number),
                                                p_rocket->getCnr(p_rocket->velocity.mach_number));
    Eigen::Vector3d inertia_target(std::pow(p_rocket->diameter, 2), std::pow(p_rocket->length, 2), std::pow(p_rocket->length, 2));
    moment_aero_dumping = p_rocket->dynamic_pressure * coefficient_aero_dumping.array() * p_rocket->area * inertia_target.array()
                            / (2.0 * p_rocket->velocity.air_body.norm()) * p_rocket->angular_velocity.array();

    return moment_aero_dumping;
};


Eigen::Vector3d forrocket::DynamicsBase::JetDampingMoment() {
    Eigen::Vector3d moment_jet_dumping;

    moment_jet_dumping << 0.0, 0.0, 0.0;

    return moment_jet_dumping;
};


Eigen::Vector3d forrocket::DynamicsBase::QuaternionDiff() {
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

