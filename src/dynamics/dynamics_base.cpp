// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_base.cpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "dynamics_base.hpp"

#include <cmath>


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

    Eigen::Vector3d moment_arm(p_rocket->length_CG - p_rocket->length_thrust, 0.0, 0.0);
    moment_thrust = p_rocket->force.thrust.cross(moment_arm);
    
    return moment_thrust;
};


Eigen::Vector3d forrocket::DynamicsBase::AeroForceMoment(Rocket* p_rocket) {
    Eigen::Vector3d moment_aero;

    Eigen::Vector3d moment_arm(p_rocket->length_CG - p_rocket->length_CP, 0.0, 0.0);
    moment_aero = p_rocket->force.aero.cross(moment_arm);
    moment_aero[0] = p_rocket->dynamic_pressure * p_rocket->Cld * p_rocket->area * p_rocket->length * p_rocket->cant_angle_fin * 4;

    return moment_aero;
};


Eigen::Vector3d forrocket::DynamicsBase::AeroDampingMoment(Rocket* p_rocket) {
    Eigen::Vector3d moment_aero_dumping;

    Eigen::Vector3d coefficient_aero_dumping(p_rocket->Clp, p_rocket->Cmq, p_rocket->Cnr);
    
    moment_aero_dumping = p_rocket->dynamic_pressure * coefficient_aero_dumping.array() * p_rocket->area * std::pow(p_rocket->length, 2)
                            / (2.0 * p_rocket->velocity.air_body.norm()) * p_rocket->angular_velocity.array();
    
    return moment_aero_dumping;
};


Eigen::Vector3d forrocket::DynamicsBase::JetDampingMoment(Rocket* p_rocket) {
    Eigen::Vector3d moment_jet_dumping;

    moment_jet_dumping << 0.0, 0.0, 0.0;

    return moment_jet_dumping;
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

