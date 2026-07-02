// ******************************************************
// Project Name    : ForRocket
// File Name       : satmo1976.cpp
// Creation Date   : 2020/03/02
//
// Copyright (c) 2020 Susumu Tanaka. All rights reserved.
// ******************************************************

#include "satmo1976.hpp"

#include <array>

double standardatmosphere1976::pi = 3.141592653589793;

// U.S. Standard Atmosphere 1976 (NOAA/NASA/USAF, 1976) defines the effective
// earth radius r0 = 6356.766 km for the geometric <-> geopotential height
// conversion (Part 1, eq. 18).
double standardatmosphere1976::us76_earth_radius = 6356.766;
// double standardatmosphere1976::equatorial_radius = 6378.1370;
// double standardatmosphere1976::lat45deg_radius = 6367.4895;
// double standardatmosphere1976::authalic_radius = 6371.0012;  // same area earth
// double standardatmosphere1976::volumetric_radius = 6371.0008;  // same valume earth
double standardatmosphere1976::earth_radius = us76_earth_radius;

double standardatmosphere1976::g0 = 9.80665;
double standardatmosphere1976::mol_weight_sealevel = 28.9644;
double standardatmosphere1976::Rstar = 8314.32;
double standardatmosphere1976::gmr = 1000.0 * g0 * mol_weight_sealevel / Rstar;

// double standardatmosphere1976::beta_viscosity = 1.458e-6;
// double standardatmosphere1976::sutherland = 110.4;
// double standardatmosphere1976::avogadro = 6.022169e26;
// double standardatmosphere1976::bolzman = 1.380622e-23;

double standardatmosphere1976::temp_sealevel = 288.15;
double standardatmosphere1976::pressure_sealevel = 101325.0;
double standardatmosphere1976::density_sealevel = 1.225;
double standardatmosphere1976::soundspeed_sealevel = 340.294;
// double standardatmosphere1976::viscosity_sealevel = 1.7894e-5;
// double standardatmosphere1976::kinamtic_viscosity_sealevel = 1.4607e-5;
// double standardatmosphere1976::kappa_sealevel = 0.025326;
// double standardatmosphere1976::area_air = 3.65e-10;

// double standardatmosphere1976::num_density_sealevel = density_sealevel * avogadro / mol_weight_sealevel;
// double standardatmosphere1976::part_speed_sealevel_sq = (8.0 / pi) * Rstar * temp_sealevel / mol_weight_sealevel;
// double standardatmosphere1976::part_spped_sealevel = std::sqrt(part_speed_sealevel_sq);
// double standardatmosphere1976::free_path_sealevel = Rstar * temp_sealevel / (std::sqrt(2.0) * pi * avogadro * area_air * area_air * pressure_sealevel);
// double standardatmosphere1976::pressure_scaleheight_sealevel = Rstar * temp_sealevel / (mol_weight_sealevel * g0);

std::array<double, 4> standardatmosphere1976::Atmosphere(double geometric_altitude) {
    // input : geometric altitude [m]
    // output: density, pressure, temeratrue, sound speed
    std::array<double, 3> res;
    if (geometric_altitude > 86.0e3) {
        res = UpperAtmosphere(geometric_altitude/1e3);
    } else {
        res = LowerAtmosphere(geometric_altitude/1e3);
    }
    return {{res[0], res[1], res[2], std::sqrt(res[2] / temp_sealevel) * soundspeed_sealevel}};
};

std::array<double, 3> standardatmosphere1976::LowerAtmosphere(double geometric_altitude) {
    using Layer = std::array<double, 8>;
    static const Layer height_array = {0.0, 11.0, 20.0, 32.0, 47.0, 51.0, 71.0, 84.852};  // [km] geopotential height
    static const Layer temperature_gradient_array = {-6.5, 0.0, 1.0, 2.8, 0.0, -2.8, -2.0, 0.0};  // [K/km] Temperature gradient
    static const Layer temperature_array = {288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946};  // [K]
    // Layer pressure_array = {101325.0, 22632.0, 5474.9, 868.02, 110.91, 66.939, 3.9564, 0.3734};  // [Pa]
    static const Layer pressure_gradient_array = {1.0, 2.233611e-1, 5.403295e-2, 8.5666784e-3, 1.0945601e-3, 6.6063531e-4, 3.9046834e-5, 3.68501e-6};

    double geopotential_height = geometric_altitude * earth_radius / (geometric_altitude + earth_radius);  // geopotential altitude

    int j = 7;  // Layer number
    int i = 0;
    for (; i < 7;) {
        int k = (i + j) / 2;
        if (geopotential_height < height_array[k]) {
            j = k;
        } else {
            i = k;
        }
        if (j <= i + 1) {
            break;
        }
    }

    double temp_gradient = temperature_gradient_array[i];
    double temp_base = temperature_array[i];
    double delta_height = geopotential_height - height_array[i];
    double temp_local = temp_base + temp_gradient * delta_height;
    double theta = temp_local / temperature_array[0];

    double delta; 
    if (temp_gradient == 0.0) {
        delta = pressure_gradient_array[i] * std::exp(-gmr * delta_height / temp_base);
    } else {
        delta = pressure_gradient_array[i] * std::pow(temp_base / temp_local, gmr / temp_gradient);
    }
    double sigma = delta / theta;

    return {{sigma * density_sealevel, delta * pressure_sealevel, theta * temp_sealevel}};
};


namespace {
    std::array<double, 25> LogArray(const std::array<double, 25>& src) {
        std::array<double, 25> res;
        for (int i=0; i < 25; ++i) {
            res[i] = std::log(src[i]);
        }
        return res;
    }
}

std::array<double, 3> standardatmosphere1976::UpperAtmosphere(double geometric_altitude) {
    using Layer = std::array<double, 25>;
    static const Layer height_array = {86.0, 93.0, 100.0, 107.0, 114.0, 121.0, 128.0, 135.0, 142.0, 150.0,
                            160.0, 170.0, 180.0, 190.0, 200.0, 220.0, 260.0, 300.0, 400.0,
                            500.0, 600.0, 700.0, 800.0, 900.0, 1000.0};
    static const Layer pressure_ratio_array = {3.6850e-6, 1.0660e-6, 3.1593e-7, 1.0611e-7, 4.3892e-8,
                                        2.3095e-8, 1.3997e-8, 9.2345e-9, 6.4440e-9, 4.4828e-9,
                                        2.9997e-9, 2.0933e-9, 1.5072e-9, 1.1118e-9, 8.3628e-10,
                                        4.9494e-10, 1.9634e-10, 8.6557e-11, 1.4328e-11, 2.9840e-12,
                                        8.1056e-13, 3.1491e-13, 1.6813e-13, 1.0731e-13, 7.4155e-14};
    static const Layer density_ratio_array = {5.680E-6, 1.632E-6, 4.575E-7, 1.341E-7, 4.061E-8,
                                    1.614e-8, 7.932e-9, 4.461e-9, 2.741e-9, 1.694e-9,
                                    1.007e-9, 6.380e-10, 4.240e-10, 2.923e-10, 2.074e-10,
                                    1.116e-10, 3.871e-11, 1.564e-11, 2.288e-12, 4.257e-13,
                                    9.279e-14, 2.506e-14, 9.272e-15, 4.701e-15, 2.907e-15};

    // Computed once: 50 std::log calls per evaluation would otherwise dominate
    // the >86 km regime.
    static const Layer log_pressure_ratio_array = LogArray(pressure_ratio_array);
    static const Layer log_density_ratio_array = LogArray(density_ratio_array);

    static const Layer delta_log_pressure_ratio_array = {-0.174061, -0.177924, -0.167029, -0.142755, -0.107859,
                                            -0.079322, -0.064664, -0.054879, -0.048260, -0.042767,
                                            -0.037854, -0.034270, -0.031543, -0.029384, -0.027632,
                                            -0.024980, -0.021559, -0.019557, -0.016735, -0.014530,
                                            -0.011314, -0.007677, -0.005169, -0.003944, -0.003612};

    static const Layer delta_log_density_ratio_array = {-0.172421, -0.182258, -0.178090, -0.176372, -0.154322,
                                            -0.113750, -0.090582, -0.075033, -0.064679, -0.056067,
                                            -0.048461, -0.043042, -0.038869, -0.035648, -0.033063,
                                            -0.029164, -0.024220, -0.021336, -0.017686, -0.016035,
                                            -0.014327, -0.011631, -0.008248, -0.005580, -0.004227};

    if (geometric_altitude >= height_array.back()) {  // >= : at exactly 1000 km the
        // binary search below would land i=24 and read height_array[i+1] out of bounds.
        return {{density_ratio_array.back() * density_sealevel, pressure_ratio_array.back() * pressure_sealevel, 1000.0}};
    }
    
    int i = 0;
    int j = 25;
    for (;i < 25;) {
        int k = (i + j) / 2;
        if (geometric_altitude < height_array[k]) {
            j = k;
        } else {
            i = k;
        }
        if (j <= i + 1) {
            break;
        }
    }

    double delta = std::exp(EvaluateCubic(height_array[i], log_pressure_ratio_array[i], delta_log_pressure_ratio_array[i],
                                            height_array[i+1], log_pressure_ratio_array[i+1], delta_log_pressure_ratio_array[i+1], geometric_altitude));
    double sigma = std::exp(EvaluateCubic(height_array[i], log_density_ratio_array[i], delta_log_density_ratio_array[i],
                                            height_array[i+1], log_density_ratio_array[i+1], delta_log_density_ratio_array[i+1], geometric_altitude));
    double temperatrue = KineticTemperature(geometric_altitude);
    return {{sigma * density_sealevel, delta * pressure_sealevel, temperatrue}};
};


double standardatmosphere1976::EvaluateCubic(double a, double fa, double fpa, double b, double fb, double fpb, double u) {
    double d = (fb - fa) / (b - a);
    double t = (u - a) / (b - a);
    double p = 1.0 - t;
    double fu = p * fa + t * fb - p * t * (b - a) * (p * (d - fpa) - t * (d - fpb));
    return fu;
};


double standardatmosphere1976::KineticTemperature(double geometric_altitude) {
    double z = geometric_altitude;

    double C1 = -76.3232;
    double C2 = 19.9429;
    double C3 = 12.0;
    double C4 = 0.01875;
    double TC = 263.1905;
    double T7 = 186.8673;
    double Z8 = 91.0;
    double Z9 = 110.0;  
    double T9 = 240.0;
    double Z10 = 120.0;
    double T10 = 360.0;
    double T12 = 1000.0;

    double t;
    double xx, yy;

    if (z <= Z8) {
        t = T7;
    } else if (z < Z9) {
        xx = (z - Z8) / C2;
        yy = std::sqrt(1.0 - xx * xx);
        t = TC + C1 * yy;
    } else if (z < Z10) {
        t = T9 + C3 * (z - Z9);
    } else {
        xx = (earth_radius + Z10) / (earth_radius + z);
        yy = (T12 - T10) * std::exp(-C4 * (z - Z10) * xx);
        t = T12 - yy;
    }
    return t;
};

    
