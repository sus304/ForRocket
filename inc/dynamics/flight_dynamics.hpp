// ******************************************************
// Project Name    : ForRocket
// File Name       : flight_dynamics.hpp
// Creation Date   : 2026/05/02
//
// Copyright (c) 2026 Susumu Tanaka. All rights reserved.
// ******************************************************

#ifndef FLIGHTDYNAMICS_HPP_
#define FLIGHTDYNAMICS_HPP_

#include "Eigen/Core"
#include "dynamics/dynamics_base.hpp"
#include "environment/air.hpp"
#include "environment/coordinate.hpp"
#include "environment/sequence_clock.hpp"
#include "environment/wind.hpp"
#include "rocket/rocket.hpp"

namespace forrocket {

class FlightDynamics : public DynamicsBase {
 public:
    enum Regime {
        kOnLauncher,
        kInAir,
        kParachute,
    };

    FlightDynamics(Rocket* rocket, SequenceClock* clock, EnvironmentWind* wind);

    void operator()(const state& x, state& dx, const double t) override;

    Regime regime() const { return regime_; }
    void SetRegime(Regime r) { regime_ = r; }

 private:
    Rocket* p_rocket_;
    SequenceClock* p_clock_;
    EnvironmentWind* p_wind_;
    Regime regime_;

    void Compute3dofOnLauncher(const state& x, state& dx, const double t);
    void Compute6dofAero(const state& x, state& dx, const double t);
    void Compute6dofProgramRate(const state& x, state& dx, const double t);
    void Compute3dofParachute(const state& x, state& dx, const double t);

    Eigen::Vector3d GravityNED(const double altitude, const Coordinate& coord);
    void SyncNavigation(const state& x, double t, Coordinate& coord);
    void UpdateAeroCoefficients(const Coordinate& coord, const EnvironmentAir& air);
    void ComputeForces(const Coordinate& coord, const EnvironmentAir& air,
                       const Eigen::Vector3d& gravity_NED);
    void ComputeMoments(double t);
};

}  // namespace forrocket

#endif
