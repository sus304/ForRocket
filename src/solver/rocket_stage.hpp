// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket_stage.hpp
// Creation Date   : 2020/01/27
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef ROCKETSTAGE_HPP_
#define ROCKETSTAGE_HPP_

#include <vector>

#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"

#include "dynamics/dynamics_base.hpp"
#include "rocket/rocket.hpp"
#include "rocket/flight_data_recorder.hpp"

namespace forrocket {
    class RocketStage {
        public:
            RocketStage(int stage_number, Rocket rocket);

            int stage_number;

            Rocket rocket;
            FlightDataRecorder fdr;
            bool enable_program_attitude;

            // ここでの時刻はX+の絶対時刻
            
            double time_start;
            double time_ignittion;

            bool enable_launcher;
            // bool on_lancher;
            double length_launcher_rail;
            
            bool enable_cutoff;
            double time_cutoff;

            bool enable_sepation;
            double time_separation;

            bool enable_despin;
            double time_despin;

            bool enable_fairing_jettson;
            double time_jettson_fairing;
            double mass_fairing;

            bool enable_parachute_open;
            double time_opan_parachute;

            double time_end;

            void FlightSequence(SequenceClock* master_clock, EnvironmentWind* wind, DynamicsBase::state& x0);

    };
}

#endif
