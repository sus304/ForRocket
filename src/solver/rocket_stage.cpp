// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_stage.cpp
// Creation Date   : 2020/01/27
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket_stage.hpp"

#include <cmath>

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "boost/numeric/odeint.hpp"

#include "dynamics/dynamics_6dof_aero.hpp"
#include "dynamics/dynamics_6dof_programrate.hpp"
#include "dynamics/dynamics_3dof_onlauncher.hpp"
#include "dynamics/dynamics_3dof_parachute.hpp"

#ifdef DEBUG
    #include <iostream>
#endif

forrocket::RocketStage::RocketStage(int stage_number, Rocket rocket) {
    this->stage_number = stage_number;
    this->rocket = rocket;
    // this->p_fdr = new FlightDataRecorder(&this->rocket);
    #ifdef DEBUG_NO
        std::cout << "RocketStage.rocket address:" << &this->rocket << " at RocketStage Constructor" << std::endl;
    #endif
};

void forrocket::RocketStage::SwitchDynamics(const double time_start, DynamicsBase* dynamics,
                                            DynamicsBase* dynamics_aero, DynamicsBase* dynamics_program) {
    delete dynamics;
    if (time_start >= time_start_attitude_controll && time_start < time_end_attitude_controll) {
        dynamics = dynamics_program;
    } else {
        dynamics = dynamics_aero;
    }
};


void forrocket::RocketStage::FlightSequence(SequenceClock* master_clock, EnvironmentWind* wind, DynamicsBase::state& x0) {
    namespace odeint = boost::numeric::odeint;
    odeint::runge_kutta_dopri5<forrocket::DynamicsBase::state> stepper;

    Dynamics6dofAero dynamics_6dof_aero(&rocket, master_clock, wind);
    Dynamics6dofProgramRate dynamics_6dof_programrate(&rocket, master_clock, wind);
    DynamicsBase* p_dynamics;

    double start, end;
    double delta = 0.01;  // 100 Hz
    double delta_on_launcher = 0.001;  // 1000 Hz
    double delta_decent_parachute = 0.01;  // 100 Hz
    DynamicsBase::state x0_in_stage = x0;

    if (time_ignittion <= time_start) {
        // 計算開始と同時に点火する場合
        rocket.IgnitionEngine(master_clock->UTC_date_init, master_clock->countup_time);
        start = time_start;
    } else {
        // 点火が計算開始より後の場合
        // Flight start から engine ignittion まで
        if (enable_program_attitude) SwitchDynamics(time_start, p_dynamics, &dynamics_6dof_aero, &dynamics_6dof_programrate);
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, time_start, time_ignittion, delta, std::ref(fdr)); // 点火までの慣性飛行計算
        start = time_ignittion;
        rocket.IgnitionEngine(master_clock->UTC_date_init, master_clock->countup_time);
    }
    // ここまでにIgnitionEngineが実行されている

    if (enable_launcher) {
        // ランチクリア時刻確定のためにコピー品で短時間回す
        Rocket rocket_on_launcher = rocket;
        SequenceClock clock_on_launcher = *master_clock; // copy
        DynamicsBase::state x0_on_launcher = x0;
        FlightDataRecorder fdr_on_launcher(&rocket_on_launcher);
        p_dynamics = new Dynamics3dofOnLauncher(&rocket_on_launcher, &clock_on_launcher);
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_on_launcher, start, start+1.0, delta_on_launcher, std::ref(fdr_on_launcher));
        fdr_on_launcher.dump_csv("test_estimate_launcher.csv");
        for (int i=0; i < fdr_on_launcher.countup_burn_time.size(); ++i) {
            double distance = fdr_on_launcher.postion[i].LLH(2) / sin(fdr_on_launcher.attitude[i].euler_angle(1));
            if (distance >= length_launcher_rail + rocket.position.LLH(2)) {
                end = delta_on_launcher * i;
                break;
            }
        }

        // ランチャ滑走の本ちゃん
        p_dynamics = new Dynamics3dofOnLauncher(&rocket, master_clock);
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, start + end, delta_on_launcher, std::ref(fdr));
        // odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, start + end, delta_on_launcher, std::ref(fdr));
        start = start + end;
        fdr.dump_csv("test_onlauncher.csv");
    }
    // ここまでに慣性飛行からの点火もしくはランチクリアまでが実行されている

    #ifdef DEBUG
        std::cout << "Launch Clear:" << fdr.postion[fdr.countup_burn_time.size()-1].LLH(2) << std::endl;
        std::cout << "fdr.rocket address:" << fdr.p_rocket << " at sequence()" << std::endl;
    #endif
    
    if (enable_cutoff) {
        // engine cutoff まで 
        if (enable_program_attitude) SwitchDynamics(start, p_dynamics, &dynamics_6dof_aero, &dynamics_6dof_programrate);
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, time_cutoff, delta, std::ref(fdr));
        start = time_cutoff;
        rocket.CutoffEngine();
    }
    // enable_cutoff=false の場合は質量減少で自動cutoffされる（Rocketクラス側）

    #ifdef DEBUG
        std::cout << "Cutoff:" << fdr.postion[fdr.postion.size()-1].LLH(2) << std::endl;/////////////////////////////////////////////////////////////////////
    #endif

    if (enable_despin) {
        if (enable_program_attitude) SwitchDynamics(start, p_dynamics, &dynamics_6dof_aero, &dynamics_6dof_programrate);
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, time_despin, delta, std::ref(fdr));
        start = time_despin;
        rocket.DeSpin();
    }

    if (enable_fairing_jettson) {
        if (enable_program_attitude) SwitchDynamics(start, p_dynamics, &dynamics_6dof_aero, &dynamics_6dof_programrate);
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, time_jettson_fairing, delta, std::ref(fdr));
        start = time_jettson_fairing;
        rocket.JettsonFairing(mass_fairing);
    }

    if (enable_sepation) {
        if (enable_program_attitude) SwitchDynamics(start, p_dynamics, &dynamics_6dof_aero, &dynamics_6dof_programrate);
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, time_separation, delta, std::ref(fdr));
        start = time_separation;
        rocket.SeparateUpperStage(mass_upper_stage);
        x0 = x0_in_stage;
    }

    if (!enable_parachute_open) {
        // 弾道で落ちるまで
        if (enable_program_attitude) SwitchDynamics(start, p_dynamics, &dynamics_6dof_aero, &dynamics_6dof_programrate);
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, time_end, delta, std::ref(fdr));
    } else {
        // パラシュート開傘
        if (enable_apogee_parachute_open) {
            // 頂点時刻確定のためにコピー品で回す
            Rocket rocket_apogee_estimate = rocket;
            SequenceClock clock_apogee_estimate = *master_clock; // copy
            DynamicsBase::state x0_apogee_estimate = x0;
            FlightDataRecorder fdr_apogee_estimate(&rocket_apogee_estimate);
            if (enable_program_attitude) {
                p_dynamics = new Dynamics6dofProgramRate(&rocket, &clock_apogee_estimate, wind);
            } else {
                p_dynamics = new Dynamics6dofAero(&rocket, &clock_apogee_estimate, wind);
            }
            odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_apogee_estimate, start, time_end, delta, std::ref(fdr_apogee_estimate));
            double max_alt = 0.0;
            for (int i=0; i < fdr_apogee_estimate.postion.size(); ++i) {
                double alt = fdr_apogee_estimate.postion[i].LLH(2);
                if (max_alt < alt) {
                    max_alt = alt;
                } else {
                    end = fdr_apogee_estimate.countup_time[i];
                    break;
                }
            }
        } else {
            end = time_open_parachute;
        }
        // 頂点までの飛行本ちゃん
        if (enable_program_attitude) SwitchDynamics(start, p_dynamics, &dynamics_6dof_aero, &dynamics_6dof_programrate);
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, end, delta, std::ref(fdr));
        start = end;

        // ここまでで頂点もしくは開傘時刻まで実行されている
        odeint::runge_kutta4<forrocket::DynamicsBase::state> stepper;
        rocket.OpenParachute();
        Dynamics3dofParachute dynamics_3dof_parachute(&rocket, master_clock, wind);

        if (exist_second_parachute) {
            odeint::integrate_const(stepper, dynamics_3dof_parachute, x0_in_stage, start, time_open_second_parachute, delta_decent_parachute, std::ref(fdr));
            rocket.OpenParachute();
            odeint::integrate_const(stepper, dynamics_3dof_parachute, x0_in_stage, time_open_second_parachute, time_end, delta_decent_parachute, std::ref(fdr));
        } else {
            odeint::integrate_const(stepper, dynamics_3dof_parachute, x0_in_stage, start, time_end, delta_decent_parachute, std::ref(fdr));
        }
    }

    fdr.dump_csv("flight_log_stage"+std::to_string(stage_number)+".csv");
};