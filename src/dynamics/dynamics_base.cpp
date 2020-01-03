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


Eigen::Vector3d forrocket::DynamicsBase::ThrustBodyCoordinate(const double thrust, const double gimbal_angle_y_axis, const double gimbal_angle_z_axis) {
    Eigen::Vector3d thrust_vector;
    
    Eigen::Vector3d gimbal_angle(std::cos(gimbal_angle_y_axis) * std::cos(gimbal_angle_y_axis),
                                std::sin(gimbal_angle_y_axis),
                                -std::sin(gimbal_angle_z_axis));
    thrust_vector << thrust * gimbal_angle;

    return thrust_vector;
};


double forrocket::DynamicsBase::getFinCantAxialForce(const double dynamic_pressure, const double CNa_fin, const double area, const double cant_angle_fin) {
    double force_axial_fin_cant = dynamic_pressure * CNa_fin * area * std::sin(cant_angle_fin);
    return force_axial_fin_cant;
};



Eigen::Vector3d forrocket::DynamicsBase::AeroForceBodyCoordinate(const double dynamic_pressure, const double area, const double CA, 
                                                                const double CNa, const double angle_of_attack, const double sideslip_angle,
                                                                const double CNa_fin, const double cant_angle_fin) {
    Eigen::Vector3d force_aero;

    double force_axial = dynamic_pressure * CA * area;
    double force_normal_y_axis = dynamic_pressure * CNa * area * sideslip_angle;
    double force_normal_z_axis = dynamic_pressure * CNa * area * angle_of_attack;
    force_aero << force_axial + getFinCantAxialForce(dynamic_pressure, CNa_fin, area, cant_angle_fin), force_normal_y_axis, force_normal_z_axis;

    return force_aero;
};


Eigen::Vector3d forrocket::DynamicsBase::GyroEffectMoment(const Eigen::Matrix3d& inertia_tensor, const Eigen::Vector3d& angular_velocity) {
    Eigen::Vector3d gyro_effect;

    Eigen::Vector3d angular_momentum = inertia_tensor * angular_velocity;
    gyro_effect = angular_velocity.cross(angular_momentum);

    return gyro_effect;
};


Eigen::Vector3d forrocket::DynamicsBase::ThrustMoment(const Eigen::Vector3d& thrust, const double length_CG, const double length_thrust) {
    Eigen::Vector3d moment_thrust;

    Eigen::Vector3d moment_arm(length_CG - length_thrust, 0.0, 0.0);
    moment_thrust = thrust.cross(moment_arm);
    
    return moment_thrust;
};


Eigen::Vector3d forrocket::DynamicsBase::AeroForceMoment(const Eigen::Vector3d& force_aero, const double length_CG, const double length_CP, 
                                                        const double dynamic_pressure, const double area, const double radius_CP_fin) {
    Eigen::Vector3d moment_aero;

    Eigen::Vector3d moment_arm(length_CG - length_CP, 0.0, 0.0);
    moment_aero = force_aero.cross(moment_arm);
    moment_aero[0] = forrocket::DynamicsBase::getFinCantAxialForce() * p_rocket->radius_CP_fin;

    return moment_aero;
};


Eigen::Vector3d forrocket::DynamicsBase::AeroDampingMoment() {
    Eigen::Vector3d moment_aero_dumping;

    Eigen::Vector3d coefficient_aero_dumping(p_rocket->Clp, p_rocket->Cmq, p_rocket->Cnr);
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

