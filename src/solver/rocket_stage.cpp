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
};

void forrocket::RocketStage::SwitchDynamics(const double time_start, DynamicsBase** dynamics,
                                            SequenceClock* master_clock, EnvironmentWind* wind) {
    // delete *dynamics;
    if (enable_program_attitude) {
        if (time_start >= time_start_attitude_controll && time_start < time_end_attitude_controll) {
            *dynamics = new Dynamics6dofProgramRate(&rocket, master_clock, wind);
            return;
        }
    }
    *dynamics = new Dynamics6dofAero(&rocket, master_clock, wind);
};


void forrocket::RocketStage::FlightSequence(SequenceClock* master_clock, EnvironmentWind* wind, DynamicsBase::state& x0) {
    namespace odeint = boost::numeric::odeint;

    // Sterpper Select
    ////////////////////////////////////
    // 4th Order Runge-Kutta Method
    odeint::runge_kutta4<forrocket::DynamicsBase::state> stepper;

    double eps_abs = 1.0e-12;
    double eps_rel = 1.0e-7;
    // 5th Order Runge-Kutta-Dormand-Prince Method : Controlled : Dense output : Internal info
    // #define CONTROLLED_STEPPER
    // using base_stepper_type = odeint::runge_kutta_dopri5<forrocket::DynamicsBase::state>;
    // auto stepper = make_controlled( eps_abs , eps_rel , base_stepper_type());

    // 8th Order Runge-Kutta-Fehlberg Method : Controled
    // using base_stepper_type = odeint::runge_kutta_fehlberg78<forrocket::DynamicsBase::state>;
    // auto stepper = make_controlled( eps_abs , eps_rel , base_stepper_type());

    DynamicsBase* p_dynamics;
    p_dynamics = new Dynamics6dofAero(&rocket, master_clock, wind);

    double start, end;
    double time_step_on_launcher = 0.001;  // 1000 Hz
    double time_step_decent_parachute = 0.1;  // 10 Hz
    fdr.ReserveCapacity(static_cast<int>((time_end - time_start) / time_step) * 1.3);

    DynamicsBase::state x0_in_stage = x0;  // x0はソルバで次段のために共有するので内部用にコピー

    if (time_ignittion <= time_start) {
        // 計算開始と同時に点火する場合
        rocket.IgnitionEngine(master_clock->UTC_date_init, master_clock->countup_time);
        start = time_start;
    } else {
        // 点火が計算開始より後の場合
        // Flight start から engine ignittion まで
        SwitchDynamics(time_start, &p_dynamics, master_clock, wind);
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, time_start, time_ignittion, time_step, std::ref(fdr)); // 点火までの慣性飛行計算
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
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_on_launcher, start, start+1.0, time_step_on_launcher, std::ref(fdr_on_launcher));
        
        # ifdef DEBUG
        fdr_on_launcher.DumpCsv("test_estimate_launcher.csv");
        # endif
        
        for (int i=0; i < fdr_on_launcher.countup_burn_time.size(); ++i) {
            double distance = (fdr_on_launcher.position[i].LLH(2) - fdr_on_launcher.position[0].LLH(2)) / sin(fdr_on_launcher.attitude[i].euler_angle(1));
            if (distance >= length_launcher_rail) {
                end = time_step_on_launcher * i;
                std::cout << distance << ' ' << fdr_on_launcher.position[i].LLH(2) << ' ' << fdr_on_launcher.position[0].LLH(2) << ' ' << sin(fdr_on_launcher.attitude[i].euler_angle(1)) << std::endl;
                std::cout << i << ' ' << end << std::endl;
                break;
            }
        }

        // ランチャ滑走の本ちゃん
        p_dynamics = new Dynamics3dofOnLauncher(&rocket, master_clock);
        #ifdef CONTROLLED_STEPPER
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        #endif
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, start + end, time_step_on_launcher, std::ref(fdr));
        start = start + end;
        #ifdef DEBUG
        fdr.DumpCsv("debug_onlauncher_log.csv");
        #endif
    }
    // ここまでに慣性飛行からの点火もしくはランチクリアまでが実行されている
    
    if (enable_cutoff) {
        // engine cutoff まで 
        SwitchDynamics(start, &p_dynamics, master_clock, wind);
        #ifdef CONTROLLED_STEPPER
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        #endif
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, time_cutoff, time_step, std::ref(fdr));
        start = time_cutoff;
        rocket.CutoffEngine();
    }
    // enable_cutoff=false の場合は質量0で自動cutoffされる（Engineクラス側）

    if (enable_despin) {
        SwitchDynamics(start, &p_dynamics, master_clock, wind);
        #ifdef CONTROLLED_STEPPER
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        #endif
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, time_despin, time_step, std::ref(fdr));
        start = time_despin;
        rocket.DeSpin();
    }

    if (enable_fairing_jettson) {
        SwitchDynamics(start, &p_dynamics, master_clock, wind);
        #ifdef CONTROLLED_STEPPER
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        #endif
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, time_jettson_fairing, time_step, std::ref(fdr));
        start = time_jettson_fairing;
        rocket.JettsonFairing(mass_fairing);
    }

    if (enable_sepation) {
        SwitchDynamics(start, &p_dynamics, master_clock, wind);
        #ifdef CONTROLLED_STEPPER
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        #endif
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, time_separation, time_step, std::ref(fdr));
        start = time_separation;
        rocket.SeparateUpperStage(mass_upper_stage);
        x0 = x0_in_stage;  // 次段のために分離情報をコピー
    }

    if (!enable_parachute_open) {
        // 弾道で落ちるまで
        SwitchDynamics(start, &p_dynamics, master_clock, wind);
        #ifdef CONTROLLED_STEPPER
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        #endif
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, time_end, time_step, std::ref(fdr));
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
            odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_apogee_estimate, start, time_end, time_step, std::ref(fdr_apogee_estimate));
            double max_alt = 0.0;
            for (int i=0; i < fdr_apogee_estimate.position.size(); ++i) {
                double alt = fdr_apogee_estimate.position[i].LLH(2);
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
        // 開傘までの慣性飛行本ちゃん
        SwitchDynamics(start, &p_dynamics, master_clock, wind);
        #ifdef CONTROLLED_STEPPER
        stepper.initialize(std::ref(*p_dynamics), x0_in_stage, start);
        #endif
        odeint::integrate_const(stepper, std::ref(*p_dynamics), x0_in_stage, start, end, time_step, std::ref(fdr));
        start = end;

        // ここまでで頂点もしくは開傘時刻まで実行されている
        odeint::runge_kutta4<forrocket::DynamicsBase::state> stepper;
        rocket.OpenParachute();
        Dynamics3dofParachute dynamics_3dof_parachute(&rocket, master_clock, wind);

        if (exist_second_parachute) {
            odeint::integrate_const(stepper, dynamics_3dof_parachute, x0_in_stage, start, time_open_second_parachute, time_step_decent_parachute, std::ref(fdr));
            rocket.OpenParachute();
            odeint::integrate_const(stepper, dynamics_3dof_parachute, x0_in_stage, time_open_second_parachute, time_end, time_step_decent_parachute, std::ref(fdr));
        } else {
            odeint::integrate_const(stepper, dynamics_3dof_parachute, x0_in_stage, start, time_end, time_step_decent_parachute, std::ref(fdr));
        }
    }

    // fdr.dump_csv("flight_log_stage"+std::to_string(stage_number)+".csv");
};