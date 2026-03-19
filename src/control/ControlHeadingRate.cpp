#include "control/ControlHeadingRate.hpp"
#include <Eigen/Dense>

using namespace liteaerosim::control;

float velWind_horiz(const KinematicState & state)
{
    // velocity w.r.t. the wind in NED frame
    Eigen::Vector3f velWind_NED_mps = state.q_nw().toRotationMatrix() * state.velocity_Wind_mps();

    return hypot(velWind_NED_mps(0), velWind_NED_mps(1));
}

float ControlHeadingRate::step(float heading_rate_command, const KinematicState& state)
{
    float lateral_accel_command     = velWind_horiz(state) * heading_rate_command;
    float lateral_accel_measurement = state.headingRate_rps() * velWind_horiz(state);

    return controller_.step(lateral_accel_command, lateral_accel_measurement);
}

void ControlHeadingRate::reset(float heading_rate_command, const KinematicState& state)
{
    float lateral_accel_command     = velWind_horiz(state) * heading_rate_command;
    float lateral_accel_measurement = state.headingRate_rps() * velWind_horiz(state);

    controller_.reset(lateral_accel_command, lateral_accel_measurement, 0.0f);
    controller_.integrator().resetTo(0.0f);
}
