// ******************************************************
// Project Name    : ForRocket
// File Name       : Stage.hpp
// Creation Date   : 2019/07/19
 
// Copyright © 2019 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef Stage_hpp_
#define Stage_hpp_

#include <iostream>
#include <vector>

#include "../lib/Eigen/Core"

namespace ForRocket
{
    class Stage {
        public:
            double mass_fuel;
            double mass_fuel_init;
            double delta_mass_fuel;
            double mass_fuel_burnout;
            
            double mass_ox;
            double mass_ox_init;
            
            double mass_prop_init;
            double mass_prop;

            double dia_throat;
            double area_throat;
            double eps;
            double area_exit;
            double dia_exit;

            double dia_fuel_out;
            double dia_fuel_port_init;
            double length_fuel;
        protected:
    };

}
#endif