#pragma once
#include "environment/AtmosphericState.hpp"
#include "environment/TurbulenceVelocity.hpp"
#include <Eigen/Dense>

namespace liteaerosim::environment {

struct EnvironmentState {
    AtmosphericState   atmosphere;
    Eigen::Vector3f    wind_NED_mps;       // steady ambient wind, NED frame (m/s)
    TurbulenceVelocity turbulence;         // continuous turbulence, body frame
    Eigen::Vector3f    gust_body_mps;      // discrete gust velocity, body frame (m/s)
};

} // namespace liteaerosim::environment
