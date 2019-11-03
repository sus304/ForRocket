// ******************************************************
// Project Name    : ForRocket
// File Name       : rocket.hpp
// Creation Date   : 2018/11/02
 
// Copyright © 2018 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef rocket.hpp
#define rocket.hpp

#include <iostream>
#include <vector>

#include "../lib/Eigen/Core"
#include <boost/numeric/odeint.hpp>

#include "interpolate.hpp"

namespace ForRocket
{

struct Rocket
{
    double length;
    double dia;
    double area;

    double dia_throat;
    double area_throat;
    double eps;
    double area_exit;
    double dia_exit;

    double dia_fuel_out;
    double dia_fuel_port_init;
    double length_fuel;
    
    double mass_dry;
    double mass_fuel;
    double mass_ox;
    double mass_fuel_init;
    double mass_ox_init;
    double mass_prop_init;
    double mass_strct;
    double mass_prop;
    double mass;
    double delta_mass_fuel;
    double mass_fuel_burnout;

    Eigen::VectorXd time_thrust_log;
    Eigen::VectorXd thrust_log;
    Interp1d thrust;
    double ref_thrust;
    double total_impulse;
    double Isp;

    double mdot_f;
    double mdot_ox;

    double Lcg_dry;
    double Lcg_ox;
    double Lcg_fuel;
    double Lcg_strct;
    double Lcg_prop;
    double Lcg_ox_init;
    double length_end2tank;

    double upper_lug;
    double lower_lug;

    double inertia_moment_dry_BODYframe_pitch;
    double inertia_moment_dry_BODYframe_roll;
    double inertia_moment_BODY_BODYframe_pitch;
    double inertia_moment_BODY_BODYframe_roll;
    double inertia_moment_strct_BODYframe_pitch;
    double inertia_moment_strct_BODYframe_roll;
    double inertia_moment_fuel_BODYframe_pitch;
    double inertia_moment_fuel_BODYframe_roll;
    double inertia_moment_fuel_FUELframe_pitch;
    double inertia_moment_fuel_FUELframe_roll;

    Eigen::VectorXd mach_CA_log;
    Eigen::VectorXd CA_log;
    Interp1d CA;

    Eigen::VectorXd mach_Lcp_log;
    Eigen::VectorXd Lcp_log;
    Interp1d Lcp;

    Eigen::VectorXd mach_CNa_log;
    Eigen::VectorXd CNa_log;
    Interp1d CNa;

    double Clp;  // roll
    double Cmq;  // pitch
    double Cnr;  // yaw

    double CdS_1st;

    bool exist_2nd;
    double CdS_2nd;
    double alt_open_2nd;

    bool timer_simulation;
    double time_open_1st;
    double time_open_2nd_min;
    double time_open_2nd_max;

    Eigen::VectorXd alt_wind_log;
    Eigen::VectorXd wind_speed_log;
    Eigen::VectorXd wind_direction_log;
    Interp1d wind_speed;
    Interp1d wind_direction;

    double azimuth_init;
    double elevation_init;
    double roll_init;
    double launcher_rail;
    


    bool auto_end;
    double time_end_flight;
    double time_step_flight;



    Rocket();

    using state = std::array<double, 14>;
    using state_parachute = std::array<double, 6>;

    void dynamics_6dof_ballistic(const state& x, const double t);
    void dynamics_3dof_parachute(const state_parachute& x, const double t);

    void operator()(const state& x, state& dx, const double t);

};

struct RocketObserver : public Rocket
{
    using state = Rocket::state;
    void operator()(const state& x, double t);
};


}
#endif