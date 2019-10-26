// ******************************************************
// Project Name    : ForRocket
// File Name       : dynamics_base.cpp
// Creation Date   : 2019/10/20
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "dynamics_base.hpp"

#include <cmath>

forrocket::DynamicsBase::DynamicsBase(Rocket* rocket) {
    p_rocket = rocket;
};


Eigen::Vector3d forrocket::DynamicsBase::ThrustBodyCoordinate() {
    Eigen::Vector3d thrust;
    
    Eigen::Vector3d gimbal_angle(std::cos(p_rocket->engine.gimbal_angle_y_axis) * std::cos(p_rocket->engine.gimbal_angle_y_axis),
                                std::sin(p_rocket->engine.gimbal_angle_y_axis),
                                -std::sin(p_rocket->engine.gimbal_angle_z_axis));
    thrust << p_rocket->engine.thrust * gimbal_angle.array();

    return thrust;
};


double forrocket::DynamicsBase::getFinCantAxialForce() {
    double force_axial_fin_cant = p_rocket->dynamic_pressure * p_rocket->CNa_fin * p_rocket->area * p_rocket->cant_angle_fin;
    return force_axial_fin_cant;
};



Eigen::Vector3d forrocket::DynamicsBase::AeroForceBodyCoordinate() {
    Eigen::Vector3d force_aero;

    double force_axial = p_rocket->dynamic_pressure * p_rocket->CA * p_rocket->area;
    double force_normal_y_axis = p_rocket->dynamic_pressure * p_rocket->CNa * p_rocket->area * p_rocket->sideslip_angle;
    double force_normal_z_axis = p_rocket->dynamic_pressure * p_rocket->CNa * p_rocket->area * p_rocket->angle_of_attack;
    force_aero << force_axial + forrocket::DynamicsBase::getFinCantAxialForce(), force_normal_y_axis, force_normal_z_axis;

    return force_aero;
};


Eigen::Vector3d forrocket::DynamicsBase::GyroEffectMoment() {
    Eigen::Vector3d gyro_effect;

    Eigen::Vector3d angular_momentum = p_rocket->inertia_tensor * p_rocket->omega;
    gyro_effect = p_rocket->omega.cross(angular_momentum);

    return gyro_effect;
};


Eigen::Vector3d forrocket::DynamicsBase::ThrustMoment() {
    Eigen::Vector3d moment_thrust;

    Eigen::Vector3d moment_arm(p_rocket->length_CG - p_rocket->length_thrust, 0.0, 0.0);
    moment_thrust = p_rocket->force_thrust.cross(moment_arm);
    
    return moment_thrust;
};


Eigen::Vector3d forrocket::DynamicsBase::AeroForceMoment() {
    Eigen::Vector3d moment_aero;

    Eigen::Vector3d moment_arm(p_rocket->length_CG - p_rocket->length_CP, 0.0, 0.0);
    moment_aero = p_rocket->force_aero.cross(moment_arm);
    moment_aero[0] += forrocket::DynamicsBase::getFinCantAxialForce() * p_rocket->radius_CP_fin;

    return moment_aero;
};


Eigen::Vector3d forrocket::DynamicsBase::AeroDampingMoment() {
    Eigen::Vector3d moment_aero_dumping;

    Eigen::Vector3d coefficient_aero_dumping(p_rocket->Clp, p_rocket->Cmq, p_rocket->Cnr);
    Eigen::Vector3d inertia_target(std::pow(p_rocket->diameter, 2), std::pow(p_rocket->length, 2), std::pow(p_rocket->length, 2));
    moment_aero_dumping = p_rocket->dynamic_pressure * coefficient_aero_dumping.array() * p_rocket->area * inertia_target.array()
                            / (2.0 * p_rocket->velocity.air_body.norm()) * p_rocket->omega.array();

    return moment_aero_dumping;
};


Eigen::Vector3d forrocket::DynamicsBase::JetDampingMoment() {
    Eigen::Vector3d moment_jet_dumping;

    moment_jet_dumping << 0.0, 0.0, 0.0;

    return moment_jet_dumping;
};

