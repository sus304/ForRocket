// ******************************************************
// Project Name    : ForRocket
// File Name       : satmo1976.hpp
// Creation Date   : 2020/03/02
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef SATMO1976_HPP_
#define SATMO1976_HPP_

#include <cmath>
#include <vector>

namespace standardatmosphere1976 {
    extern double pi;

    extern double polar_radius;
    // extern double equatorial_radius;
    // extern double lat45deg_radius;
    // extern double authalic_radius;  // same area earth
    // extern double volumetric_radius;  // same valume earth
    extern double earth_radius;

    extern double g0;
    extern double mol_weight_sealevel;
    extern double Rstar;
    extern double gmr;

    // extern double beta_viscosity;
    // extern double sutherland;
    // extern double avogadro;
    // extern double bolzman;

    extern double temp_sealevel;
    extern double pressure_sealevel;
    extern double density_sealevel;
    extern double soundspeed_sealevel;
    // extern double viscosity_sealevel;
    // extern double kinamtic_viscosity_sealevel;
    // extern double kappa_sealevel;
    // extern double area_air;

    // extern double num_density_sealevel;
    // extern double part_speed_sealevel_sq;
    // extern double part_spped_sealevel;
    // extern double free_path_sealevel;
    // extern double pressure_scaleheight_sealevel;

    
    std::vector<double> Atmosphere(double geometric_altitude);
    std::vector<double> LowerAtmosphere(double geometric_altitude);
    std::vector<double> UpperAtmosphere(double geometric_altitude);
    double EvaluateCubic(double a, double fa, double fpa, double b, double fb, double fpb, double u);
    double KineticTemperature(double geometric_altitude);
}

#endif
