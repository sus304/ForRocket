// ******************************************************
// Project Name    : ForRocket
// File Name       : Rocket.hpp
// Creation Date   : 2019/07/19
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef Rocket_hpp_
#define Rocket_hpp_

#include <iostream>
#include <vector>

#include "../lib/Eigen/Core"

#include "Stage.hpp"
#include "Payload.hpp"

#include "FlightDataRecorder.hpp"
#include "FlightController.hpp"

namespace ForRocket
{
    class Rocket{
        public:
            Rocket();
            Rocket(std::vector<std::string> stage_json_filename);

            void add_stage(Stage obj);
            void add_payload(Payload obj);

            void separate_stage();
            void jettison_fairing();
            void ignition_engine();
            void cutoff_engine();

            void release_parachute();

        protected:
            std::vector<Stage> stages;
            std::vector<Payload> payloads;

            FlightDataRecorder fdr;
            FlightController fc;

            double mass;
            double mass_fairing;
            Eigen::Matrix3d inertia_tensor;
            
    };

}
#endif