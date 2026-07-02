// ******************************************************
// Project Name    : ForRocket
// File Name       : flight_dynamics.cpp
// Creation Date   : 2026/05/02
//
// Copyright (c) 2026 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "flight_dynamics.hpp"

#include <cmath>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "environment/air.hpp"
#include "environment/coordinate.hpp"
#include "environment/gravity.hpp"


forrocket::FlightDynamics::FlightDynamics(Rocket* rocket, SequenceClock* clock, EnvironmentWind* wind)
    : p_rocket_(rocket), p_clock_(clock), p_wind_(wind), regime_(kInAir) {}


void forrocket::FlightDynamics::operator()(const state& x, state& dx, const double t) {
    switch (regime_) {
        case kOnLauncher:
            Compute3dofOnLauncher(x, dx, t);
            break;
        case kParachute:
            Compute3dofParachute(x, dx, t);
            break;
        case kInAir:
        default:
            if (p_rocket_->enable_program_attitude
                    && t >= p_rocket_->time_start_attitude_control
                    && t < p_rocket_->time_end_attitude_control) {
                Compute6dofProgramRate(x, dx, t);
            } else {
                Compute6dofAero(x, dx, t);
            }
            break;
    }
}


void forrocket::FlightDynamics::SyncNavigation(const state& x, const double t, Coordinate& coord) {
    p_clock_->SyncSolverTime(t);
    p_rocket_->burn_clock.SyncSolverTime(t);

    coord.setECI2ECEF(t);

    p_rocket_->position.ECI = Eigen::Map<const Eigen::Vector3d>(x.data());
    p_rocket_->position.ECEF = coord.dcm.ECI2ECEF * p_rocket_->position.ECI;
    p_rocket_->position.LLH = coord.ECEF2LLH(p_rocket_->position.ECEF);

    coord.setECEF2NED(p_rocket_->position.LLH);

    p_rocket_->velocity.ECI = Eigen::Map<const Eigen::Vector3d>(x.data() + 3);
    p_rocket_->velocity.ECEF = coord.dcm.ECI2ECEF
            * (p_rocket_->velocity.ECI - coord.dcm.EarthRotate * p_rocket_->position.ECI);
    p_rocket_->velocity.NED = coord.dcm.ECEF2NED * p_rocket_->velocity.ECEF;
}


void forrocket::FlightDynamics::UpdateAeroCoefficients(const Coordinate& coord, const EnvironmentAir& air) {
    const double altitude = p_rocket_->position.LLH[2];
    p_rocket_->velocity.air_body = coord.dcm.NED2body
            * (p_rocket_->velocity.NED - p_wind_->getNED(altitude));
    p_rocket_->dynamic_pressure = 0.5 * air.density * std::pow(p_rocket_->velocity.air_body.norm(), 2);
    p_rocket_->velocity.mach_number = p_rocket_->velocity.air_body.norm() / air.speed_of_sound;

    p_rocket_->inertia_tensor = p_rocket_->getInertiaTensor();
    p_rocket_->length_CG = p_rocket_->getLengthCG();
    p_rocket_->y_CG = p_rocket_->getYCG();
    p_rocket_->z_CG = p_rocket_->getZCG();
    p_rocket_->length_CP = p_rocket_->getLengthCP(p_rocket_->velocity.mach_number);
    p_rocket_->CA = p_rocket_->getCA(p_rocket_->velocity.mach_number);
    p_rocket_->CNa = p_rocket_->getCNa(p_rocket_->velocity.mach_number);
    p_rocket_->Cld = p_rocket_->getCld(p_rocket_->velocity.mach_number);
    p_rocket_->Clp = p_rocket_->getClp(p_rocket_->velocity.mach_number);
    p_rocket_->Cmq = p_rocket_->getCmq(p_rocket_->velocity.mach_number);
    p_rocket_->Cnr = p_rocket_->getCnr(p_rocket_->velocity.mach_number);

    const double norm = p_rocket_->velocity.air_body.norm();
    if (norm <= 0.0) {
        p_rocket_->angle_of_attack = 0.0;
        p_rocket_->sideslip_angle = 0.0;
    } else {
        p_rocket_->angle_of_attack = std::asin(std::max(-1.0, std::min(1.0, p_rocket_->velocity.air_body[2] / norm)));
        p_rocket_->sideslip_angle = std::asin(std::max(-1.0, std::min(1.0, -p_rocket_->velocity.air_body[1] / norm)));
    }
}


Eigen::Vector3d forrocket::FlightDynamics::GravityNED(const double altitude, const Coordinate& coord) {
    if (p_rocket_->gravity_model_j2) {
        // 質点+J2 重力（地心半径・緯度依存・扁平項）を ECEF で評価して NED へ変換
        return coord.dcm.ECEF2NED * gravityECEF(p_rocket_->position.ECEF);
    }
    return Eigen::Vector3d(0.0, 0.0, gravity(altitude));
}


void forrocket::FlightDynamics::ComputeForces(const Coordinate& coord, const EnvironmentAir& air,
                                              const Eigen::Vector3d& gravity_NED) {
    p_rocket_->force.thrust = p_rocket_->getThrust(air.pressure);
    p_rocket_->force.aero = AeroForce(p_rocket_);
    p_rocket_->force.gravity = (coord.dcm.NED2body * gravity_NED) * p_rocket_->mass.Sum();

    p_rocket_->acceleration.body = p_rocket_->force.Sum() / p_rocket_->mass.Sum();
    p_rocket_->acceleration.ECI = coord.dcm.ECEF2ECI
            * (coord.dcm.NED2ECEF * (coord.dcm.body2NED * p_rocket_->acceleration.body));
}


void forrocket::FlightDynamics::ComputeMoments(double t) {
    p_rocket_->moment.gyro = GyroEffectMoment(p_rocket_);
    p_rocket_->moment.thrust = ThrustMoment(p_rocket_);
    p_rocket_->moment.aero_force = AeroForceMoment(p_rocket_);
    p_rocket_->moment.aero_dumping = AeroDampingMoment(p_rocket_);
    p_rocket_->moment.jet_dumping = JetDampingMoment(p_rocket_);
    p_rocket_->moment.gas_jet = GasJetMoment(p_rocket_, t);
    p_rocket_->angular_acceleration = p_rocket_->inertia_tensor.inverse() * p_rocket_->moment.Sum();
}


void forrocket::FlightDynamics::Compute3dofOnLauncher(const state& x, state& dx, const double t) {
    Coordinate coordinate;

    p_clock_->SyncSolverTime(t);
    p_rocket_->burn_clock.SyncSolverTime(t);

    coordinate.setECI2ECEF(t);

    p_rocket_->position.ECI = Eigen::Map<const Eigen::Vector3d>(x.data());
    p_rocket_->position.ECEF = coordinate.dcm.ECI2ECEF * p_rocket_->position.ECI;
    p_rocket_->position.LLH = coordinate.ECEF2LLH(p_rocket_->position.ECEF);

    coordinate.setECEF2NED(p_rocket_->position.LLH);

    p_rocket_->velocity.ECI = Eigen::Map<const Eigen::Vector3d>(x.data() + 3);
    p_rocket_->velocity.ECEF = coordinate.dcm.ECI2ECEF
            * (p_rocket_->velocity.ECI - coordinate.dcm.EarthRotate * p_rocket_->position.ECI);
    p_rocket_->velocity.NED = coordinate.dcm.ECEF2NED * p_rocket_->velocity.ECEF;

    p_rocket_->attitude.quaternion =
            Eigen::Map<const Eigen::Vector4d>(x.data() + 6).normalized();

    coordinate.setNED2Body(p_rocket_->attitude.quaternion);
    p_rocket_->attitude.euler_angle = coordinate.EulerAngle();

    p_rocket_->mass.propellant = x[13];

    double altitude = p_rocket_->position.LLH[2];
    EnvironmentAir air(altitude);
    Eigen::Vector3d gravity_NED = GravityNED(altitude, coordinate);

    // ランチャ上は風の影響を受けない
    p_rocket_->velocity.air_body = coordinate.dcm.NED2body * p_rocket_->velocity.NED;
    p_rocket_->dynamic_pressure = 0.5 * air.density * std::pow(p_rocket_->velocity.air_body.norm(), 2);
    p_rocket_->velocity.mach_number = p_rocket_->velocity.air_body.norm() / air.speed_of_sound;

    p_rocket_->inertia_tensor = p_rocket_->getInertiaTensor();
    p_rocket_->length_CG = p_rocket_->getLengthCG();
    p_rocket_->y_CG = p_rocket_->getYCG();
    p_rocket_->z_CG = p_rocket_->getZCG();
    p_rocket_->length_CP = p_rocket_->getLengthCP(p_rocket_->velocity.mach_number);
    p_rocket_->CA = p_rocket_->getCA(p_rocket_->velocity.mach_number);
    p_rocket_->CNa = p_rocket_->getCNa(p_rocket_->velocity.mach_number);
    p_rocket_->Cld = p_rocket_->getCld(p_rocket_->velocity.mach_number);
    p_rocket_->Clp = p_rocket_->getClp(p_rocket_->velocity.mach_number);
    p_rocket_->Cmq = p_rocket_->getCmq(p_rocket_->velocity.mach_number);
    p_rocket_->Cnr = p_rocket_->getCnr(p_rocket_->velocity.mach_number);

    p_rocket_->angle_of_attack = 0.0;
    p_rocket_->sideslip_angle = 0.0;

    // ランチャ上は body x 軸方向のみ
    Eigen::Vector3d force_aero;
    force_aero << -p_rocket_->dynamic_pressure * p_rocket_->CA * p_rocket_->area, 0.0, 0.0;

    p_rocket_->force.thrust = p_rocket_->getThrust(air.pressure);
    p_rocket_->force.thrust(1) = 0.0;
    p_rocket_->force.thrust(2) = 0.0;
    p_rocket_->force.aero = force_aero;
    p_rocket_->force.gravity = (coordinate.dcm.NED2body * gravity_NED) * p_rocket_->mass.Sum();
    p_rocket_->force.gravity(1) = 0.0;
    p_rocket_->force.gravity(2) = 0.0;

    // ランチャ・ラグ間の摩擦
    const double friction_coefficient = p_rocket_->friction_coefficient_launcher;
    double friction_force = p_rocket_->mass.Sum() * gravity_NED[2]
            * std::cos(p_rocket_->attitude.euler_angle(1)) * friction_coefficient;
    p_rocket_->force.thrust(0) -= friction_force;

    p_rocket_->acceleration.body = p_rocket_->force.Sum() / p_rocket_->mass.Sum();
    p_rocket_->acceleration.ECI = coordinate.dcm.ECEF2ECI
            * (coordinate.dcm.NED2ECEF * (coordinate.dcm.body2NED * p_rocket_->acceleration.body));
    if (p_rocket_->acceleration.body(0) < 0.0) {
        p_rocket_->acceleration.body << 0.0, 0.0, 0.0;
        p_rocket_->acceleration.ECI << 0.0, 0.0, 0.0;
    }

    dx[0] = p_rocket_->velocity.ECI[0];
    dx[1] = p_rocket_->velocity.ECI[1];
    dx[2] = p_rocket_->velocity.ECI[2];
    dx[3] = p_rocket_->acceleration.ECI[0];
    dx[4] = p_rocket_->acceleration.ECI[1];
    dx[5] = p_rocket_->acceleration.ECI[2];
    dx[6] = 0.0;
    dx[7] = 0.0;
    dx[8] = 0.0;
    dx[9] = 0.0;
    dx[10] = 0.0;
    dx[11] = 0.0;
    dx[12] = 0.0;
    dx[13] = -p_rocket_->engine.mdot_prop;
}


void forrocket::FlightDynamics::Compute6dofAero(const state& x, state& dx, const double t) {
    Coordinate coordinate;
    SyncNavigation(x, t, coordinate);

    p_rocket_->attitude.quaternion =
            Eigen::Map<const Eigen::Vector4d>(x.data() + 6).normalized();
    coordinate.setNED2Body(p_rocket_->attitude.quaternion);
    p_rocket_->attitude.euler_angle = coordinate.EulerAngle();
    p_rocket_->angular_velocity = Eigen::Map<const Eigen::Vector3d>(x.data() + 10);
    p_rocket_->mass.propellant = x[13];

    const double altitude = p_rocket_->position.LLH[2];
    const EnvironmentAir air(altitude);
    const Eigen::Vector3d gravity_NED = GravityNED(altitude, coordinate);

    UpdateAeroCoefficients(coordinate, air);
    ComputeForces(coordinate, air, gravity_NED);
    ComputeMoments(t);

    p_rocket_->quaternion_dot = 0.5 * (QuaternionDiff(p_rocket_) * p_rocket_->attitude.quaternion);

    dx[0] = p_rocket_->velocity.ECI[0];
    dx[1] = p_rocket_->velocity.ECI[1];
    dx[2] = p_rocket_->velocity.ECI[2];
    dx[3] = p_rocket_->acceleration.ECI[0];
    dx[4] = p_rocket_->acceleration.ECI[1];
    dx[5] = p_rocket_->acceleration.ECI[2];
    dx[6] = p_rocket_->quaternion_dot[0];
    dx[7] = p_rocket_->quaternion_dot[1];
    dx[8] = p_rocket_->quaternion_dot[2];
    dx[9] = p_rocket_->quaternion_dot[3];
    dx[10] = p_rocket_->angular_acceleration[0];
    dx[11] = p_rocket_->angular_acceleration[1];
    dx[12] = p_rocket_->angular_acceleration[2];
    dx[13] = -p_rocket_->engine.mdot_prop;
}


void forrocket::FlightDynamics::Compute6dofProgramRate(const state& x, state& dx, const double t) {
    Coordinate coordinate;
    SyncNavigation(x, t, coordinate);

    if (p_rocket_->attitude_program_config.mode_rate) {
        // レートモード: 制御軸は指定レート、自由軸はstateの角速度を使用
        p_rocket_->attitude.quaternion = Eigen::Map<const Eigen::Vector4d>(x.data() + 6).normalized();
        coordinate.setNED2Body(p_rocket_->attitude.quaternion);
        p_rocket_->attitude.euler_angle = coordinate.EulerAngle();

        Eigen::Vector3d euler_rates = p_rocket_->getAttitudeRate();
        if (!p_rocket_->attitude_program_config.enable_yaw)   euler_rates[0] = 0.0;
        if (!p_rocket_->attitude_program_config.enable_pitch) euler_rates[1] = 0.0;
        if (!p_rocket_->attitude_program_config.enable_roll)  euler_rates[2] = 0.0;

        // ZYX オイラー角レート -> 機体角速度 (p, q, r)
        const double phi   = p_rocket_->attitude.euler_angle[2];
        const double theta = p_rocket_->attitude.euler_angle[1];
        Eigen::Vector3d prescribed_omega;
        prescribed_omega[0] = euler_rates[2] - euler_rates[0] * std::sin(theta);
        prescribed_omega[1] = euler_rates[1] * std::cos(phi) + euler_rates[0] * std::cos(theta) * std::sin(phi);
        prescribed_omega[2] = -euler_rates[1] * std::sin(phi) + euler_rates[0] * std::cos(theta) * std::cos(phi);

        // 自由軸はstateの角速度、制御軸は指定値に上書き
        p_rocket_->angular_velocity = Eigen::Map<const Eigen::Vector3d>(x.data() + 10);
        if (p_rocket_->attitude_program_config.enable_roll)  p_rocket_->angular_velocity[0] = prescribed_omega[0];
        if (p_rocket_->attitude_program_config.enable_pitch) p_rocket_->angular_velocity[1] = prescribed_omega[1];
        if (p_rocket_->attitude_program_config.enable_yaw)   p_rocket_->angular_velocity[2] = prescribed_omega[2];
    } else {
        // 角度モード: 制御軸の姿勢角を上書き、自由軸はstateのクォータニオンを維持
        p_rocket_->attitude.quaternion = Eigen::Map<const Eigen::Vector4d>(x.data() + 6).normalized();
        coordinate.setNED2Body(p_rocket_->attitude.quaternion);
        Eigen::Vector3d euler = coordinate.EulerAngle();

        Eigen::Vector3d prescribed = p_rocket_->getAttitude();
        if (p_rocket_->attitude_program_config.enable_yaw)   euler[0] = prescribed[0];
        if (p_rocket_->attitude_program_config.enable_pitch) euler[1] = prescribed[1];
        if (p_rocket_->attitude_program_config.enable_roll)  euler[2] = prescribed[2];

        p_rocket_->attitude.euler_angle = euler;
        coordinate.setNED2Body(euler);
        p_rocket_->attitude.quaternion = coordinate.Quaternion(euler);

        // 制御軸は角速度ゼロ（姿勢角固定）、自由軸はstateの角速度
        p_rocket_->angular_velocity = Eigen::Map<const Eigen::Vector3d>(x.data() + 10);
        if (p_rocket_->attitude_program_config.enable_roll)  p_rocket_->angular_velocity[0] = 0.0;
        if (p_rocket_->attitude_program_config.enable_pitch) p_rocket_->angular_velocity[1] = 0.0;
        if (p_rocket_->attitude_program_config.enable_yaw)   p_rocket_->angular_velocity[2] = 0.0;
    }

    // 制御・自由軸の混合角速度からクォータニオン微分を計算
    p_rocket_->quaternion_dot = 0.5 * (QuaternionDiff(p_rocket_) * p_rocket_->attitude.quaternion);

    p_rocket_->mass.propellant = x[13];

    const double altitude = p_rocket_->position.LLH[2];
    const EnvironmentAir air(altitude);
    const Eigen::Vector3d gravity_NED = GravityNED(altitude, coordinate);

    UpdateAeroCoefficients(coordinate, air);
    ComputeForces(coordinate, air, gravity_NED);
    ComputeMoments(t);

    // 制御軸の角加速度はゼロ（角速度は制御系が決定）
    if (p_rocket_->attitude_program_config.enable_roll)  p_rocket_->angular_acceleration[0] = 0.0;
    if (p_rocket_->attitude_program_config.enable_pitch) p_rocket_->angular_acceleration[1] = 0.0;
    if (p_rocket_->attitude_program_config.enable_yaw)   p_rocket_->angular_acceleration[2] = 0.0;

    dx[0] = p_rocket_->velocity.ECI[0];
    dx[1] = p_rocket_->velocity.ECI[1];
    dx[2] = p_rocket_->velocity.ECI[2];
    dx[3] = p_rocket_->acceleration.ECI[0];
    dx[4] = p_rocket_->acceleration.ECI[1];
    dx[5] = p_rocket_->acceleration.ECI[2];
    dx[6] = p_rocket_->quaternion_dot[0];
    dx[7] = p_rocket_->quaternion_dot[1];
    dx[8] = p_rocket_->quaternion_dot[2];
    dx[9] = p_rocket_->quaternion_dot[3];
    dx[10] = p_rocket_->angular_acceleration[0];
    dx[11] = p_rocket_->angular_acceleration[1];
    dx[12] = p_rocket_->angular_acceleration[2];
    dx[13] = -p_rocket_->engine.mdot_prop;
}


void forrocket::FlightDynamics::Compute3dofParachute(const state& x, state& dx, const double t) {
    Coordinate coordinate;

    p_clock_->SyncSolverTime(t);
    p_rocket_->burn_clock.SyncSolverTime(t);

    coordinate.setECI2ECEF(t);

    p_rocket_->position.ECI = Eigen::Map<const Eigen::Vector3d>(x.data());
    p_rocket_->position.ECEF = coordinate.dcm.ECI2ECEF * p_rocket_->position.ECI;
    p_rocket_->position.LLH = coordinate.ECEF2LLH(p_rocket_->position.ECEF);

    coordinate.setECEF2NED(p_rocket_->position.LLH);

    p_rocket_->velocity.ECI = Eigen::Map<const Eigen::Vector3d>(x.data() + 3);
    p_rocket_->velocity.ECEF = coordinate.dcm.ECI2ECEF
            * (p_rocket_->velocity.ECI - coordinate.dcm.EarthRotate * p_rocket_->position.ECI);
    p_rocket_->velocity.NED = coordinate.dcm.ECEF2NED * p_rocket_->velocity.ECEF;

    double altitude = p_rocket_->position.LLH[2];
    EnvironmentAir air(altitude);
    Eigen::Vector3d gravity_NED = GravityNED(altitude, coordinate);

    Eigen::Vector3d wind_NED = p_wind_->getNED(altitude);
    Eigen::Vector3d v_air_NED = p_rocket_->velocity.NED - wind_NED;
    Eigen::Vector3d drag_NED = -0.5 * air.density * v_air_NED.norm() * p_rocket_->CdS_parachute * v_air_NED;
    Eigen::Vector3d acceleration_NED = drag_NED / p_rocket_->mass.Sum() + gravity_NED;
    p_rocket_->acceleration.ECI = coordinate.dcm.ECEF2ECI * (coordinate.dcm.NED2ECEF * acceleration_NED);

    dx[0] = p_rocket_->velocity.ECI[0];
    dx[1] = p_rocket_->velocity.ECI[1];
    dx[2] = p_rocket_->velocity.ECI[2];
    dx[3] = p_rocket_->acceleration.ECI[0];
    dx[4] = p_rocket_->acceleration.ECI[1];
    dx[5] = p_rocket_->acceleration.ECI[2];
    dx[6] = 0.0;
    dx[7] = 0.0;
    dx[8] = 0.0;
    dx[9] = 0.0;
    dx[10] = 0.0;
    dx[11] = 0.0;
    dx[12] = 0.0;
    dx[13] = 0.0;
}
