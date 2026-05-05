// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_stage.cpp
// Creation Date   : 2020/01/27
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "rocket_stage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "boost/numeric/odeint.hpp"

#include "dynamics/flight_dynamics.hpp"
#include "solver/flight_event.hpp"

#ifdef DEBUG
#include <iostream>
#endif


forrocket::RocketStage::RocketStage(int stage_number, Rocket rocket) {
    this->stage_number = stage_number;
    this->rocket = rocket;
    this->separated = false;
    this->time_at_separation = 0.0;
    this->state_at_separation.fill(0.0);
}


void forrocket::RocketStage::FlightSequence(SequenceClock* master_clock,
                                            EnvironmentWind* wind,
                                            DynamicsBase::state& x0) {
    namespace odeint = boost::numeric::odeint;
    typedef DynamicsBase::state state_t;

    // ------------------------------------------------------------------
    // 1. イベントリスト構築（時刻順ソート）
    // ------------------------------------------------------------------
    std::vector<std::unique_ptr<FlightEvent> > events;

    events.push_back(std::unique_ptr<FlightEvent>(
            new IgnitionEvent(time_ignittion, master_clock)));

    if (enable_cutoff) {
        events.push_back(std::unique_ptr<FlightEvent>(new CutoffEvent(time_cutoff)));
    }
    if (enable_despin) {
        events.push_back(std::unique_ptr<FlightEvent>(new DespinEvent(time_despin)));
    }
    if (enable_fairing_jettson) {
        events.push_back(std::unique_ptr<FlightEvent>(
                new JettisonFairingEvent(time_jettson_fairing, mass_fairing)));
    }
    if (enable_sepation) {
        SeparationEvent::Callback on_separate =
                [this](double t, const state_t& s) {
                    this->separated = true;
                    this->time_at_separation = t;
                    this->state_at_separation = s;
                };
        events.push_back(std::unique_ptr<FlightEvent>(
                new SeparationEvent(time_separation, mass_upper_stage, on_separate)));
    }

    bool apogee_pending = false;
    if (enable_parachute_open) {
        if (enable_apogee_parachute_open) {
            apogee_pending = true;
        } else {
            events.push_back(std::unique_ptr<FlightEvent>(
                    new ParachuteOpenEvent(time_open_parachute)));
        }
        if (exist_second_parachute) {
            events.push_back(std::unique_ptr<FlightEvent>(
                    new ParachuteOpenEvent(time_open_second_parachute)));
        }
    }

    std::sort(events.begin(), events.end(),
              [](const std::unique_ptr<FlightEvent>& a,
                 const std::unique_ptr<FlightEvent>& b) {
                  return a->Time() < b->Time();
              });

    // ------------------------------------------------------------------
    // 2. ダイナミクスとステッパ準備
    // ------------------------------------------------------------------
    FlightDynamics dynamics(&rocket, master_clock, wind);
    if (enable_launcher) {
        dynamics.SetRegime(FlightDynamics::kOnLauncher);
    } else {
        dynamics.SetRegime(FlightDynamics::kInAir);
    }

    const double eps_abs = 1.0e-9;
    const double eps_rel = 1.0e-7;
    auto stepper = odeint::make_dense_output(
            eps_abs, eps_rel,
            odeint::runge_kutta_dopri5<state_t>());

    fdr.ReserveCapacity(static_cast<int>((time_end - time_start) / time_step) * 1.3);

    // ランチャ滑走距離計算用の初期高度（射点の絶対高度）
    const double altitude_init = rocket.position.LLH(2);

    state_t x = x0;
    std::size_t next_event_idx = 0;

    // ------------------------------------------------------------------
    // 3. プリループ：t_start 以前/同時刻に発火するイベントを先に処理
    // ------------------------------------------------------------------
    while (next_event_idx < events.size()
           && events[next_event_idx]->Time() <= time_start) {
        events[next_event_idx]->Apply(rocket, x);
        if (rocket.CdS_parachute > 0.0
                && dynamics.regime() != FlightDynamics::kParachute) {
            dynamics.SetRegime(FlightDynamics::kParachute);
            apogee_pending = false;
        }
        ++next_event_idx;
    }
    x0 = x;

    stepper.initialize(x, time_start, time_step);

    // 頂点検出用の前回 NED 速度 z 成分（NaN は未初期化）
    double prev_velocity_NED_down = std::numeric_limits<double>::quiet_NaN();

    // ------------------------------------------------------------------
    // 4. メイン積分ループ
    // ------------------------------------------------------------------
    while (stepper.current_time() < time_end) {
        stepper.do_step(std::ref(dynamics));
        double t_curr = stepper.current_time();
        double t_prev = stepper.previous_time();
        state_t x_curr = stepper.current_state();

        // 4-1. 時刻イベント発火
        bool stepper_reinitialized = false;
        while (next_event_idx < events.size()
               && events[next_event_idx]->Time() <= t_curr) {
            double t_event = events[next_event_idx]->Time();
            state_t x_event;
            stepper.calc_state(t_event, x_event);
            events[next_event_idx]->Apply(rocket, x_event);

            if (rocket.CdS_parachute > 0.0
                    && dynamics.regime() != FlightDynamics::kParachute) {
                dynamics.SetRegime(FlightDynamics::kParachute);
                apogee_pending = false;
            }

            stepper.initialize(x_event, t_event, time_step);
            stepper_reinitialized = true;
            ++next_event_idx;
        }
        if (stepper_reinitialized) {
            t_curr = stepper.current_time();
            x_curr = stepper.current_state();
            // 不連続点直後はダイナミクスを再評価して状態変数を同期
            state_t dx_dummy;
            dynamics(x_curr, dx_dummy, t_curr);
        }

        // 4-2. ランチクリア検出（条件型）
        if (dynamics.regime() == FlightDynamics::kOnLauncher) {
            double altitude_change = rocket.position.LLH(2) - altitude_init;
            double sin_elv = std::sin(rocket.attitude.euler_angle(1));
            if (sin_elv > 1.0e-6) {
                double distance = altitude_change / sin_elv;
                if (distance >= length_launcher_rail) {
                    dynamics.SetRegime(FlightDynamics::kInAir);
                    rocket.time_launch_clear = t_curr;
                    stepper.initialize(x_curr, t_curr, time_step);
                }
            }
        }

        // 4-3. 頂点パラシュート開傘検出（条件型）
        if (apogee_pending && dynamics.regime() == FlightDynamics::kInAir) {
            double curr_velocity_NED_down = rocket.velocity.NED(2);
            // NED の z 成分は下向き正なので、上昇中(<0)から下降(>=0)へ反転で頂点
            if (!std::isnan(prev_velocity_NED_down)
                    && prev_velocity_NED_down < 0.0
                    && curr_velocity_NED_down >= 0.0) {
                double t_apogee;
                double dv = curr_velocity_NED_down - prev_velocity_NED_down;
                if (std::abs(dv) > 1.0e-12) {
                    double frac = -prev_velocity_NED_down / dv;
                    t_apogee = t_prev + frac * (t_curr - t_prev);
                } else {
                    t_apogee = t_curr;
                }
                state_t x_apogee;
                stepper.calc_state(t_apogee, x_apogee);
                ParachuteOpenEvent apogee_event(t_apogee);
                apogee_event.Apply(rocket, x_apogee);
                dynamics.SetRegime(FlightDynamics::kParachute);
                apogee_pending = false;

                stepper.initialize(x_apogee, t_apogee, time_step);
                t_curr = stepper.current_time();
                x_curr = stepper.current_state();
                state_t dx_dummy;
                dynamics(x_curr, dx_dummy, t_curr);
            }
            prev_velocity_NED_down = curr_velocity_NED_down;
        }

        // 4-4. 地面衝突終了
        if (rocket.position.LLH(2) < 0.0) {
            break;
        }

        // 4-5. ロギング
        fdr(x_curr, t_curr);
    }

    // ------------------------------------------------------------------
    // 5. 段間引き継ぎ
    // ------------------------------------------------------------------
    if (separated) {
        x0 = state_at_separation;
    }
}
