#include "control/ControlVerticalSpeed.hpp"
#include <cmath>

using namespace liteaerosim::control;

float velWind_horiz(const KinematicState & state)
{
    // velocity w.r.t. the wind in NED frame
    Eigen::Vector3f velWind_NED_mps = state.q_nw().toRotationMatrix() * state.velocity_Wind_mps();

    return hypot(velWind_NED_mps(0), velWind_NED_mps(1));
}

float ControlVerticalSpeed::step(float command, const KinematicState& state)
{
    controller_.setUnwrapInputs(false);

    float gamma_command     = atan2(command, velWind_horiz(state));  // positive up
    float gamma_measurement = atan2(-state.velocity_NED_mps()(2), velWind_horiz(state));
    float gamma_dot         = state.pitchRate_rps() - state.alphaDot() * cos(state.roll());

    float gamma_rate_command = controller_.step(gamma_command, gamma_measurement, gamma_dot);

    return gamma_rate_command * state.velocity_Wind_mps().norm();
}

void ControlVerticalSpeed::reset(float command, const KinematicState& state)
{
    controller_.setUnwrapInputs(false);

    float gamma_command     = atan2(command, velWind_horiz(state));  // positive up
    float gamma_measurement = atan2(-state.velocity_NED_mps()(2), velWind_horiz(state));
    controller_.reset(gamma_command, gamma_measurement, 0.0f);
}
