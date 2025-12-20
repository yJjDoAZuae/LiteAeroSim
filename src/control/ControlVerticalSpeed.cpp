#include "control/ControlVerticalSpeed.hpp"
#include <cmath>

using namespace Control;

float ControlVerticalSpeed::step(float verticalSpeedCmdIn, const KinematicState & state)
{
    pid.unwrapInputs = false;

    // velocity w.r.t. the wind in NED frame
    Eigen::Vector3f velWind_NED_mps = state.q_nw().toRotationMatrix() * state.velocity_Wind_mps();

    float TAS = velWind_NED_mps.norm();

    float velWind_horiz_mps = hypot(velWind_NED_mps(1), velWind_NED_mps(2));

    float gammaCmd = atan2(verticalSpeedCmdIn, velWind_horiz_mps); // positive up
    float gammaMeas = atan2(-state.velocity_NED_mps()(2), velWind_horiz_mps);
    float gammaRateCmd = pid.step(gammaCmd, gammaMeas, state.pitchRate_rps());

    return gammaRateCmd*TAS;
}

void ControlVerticalSpeed::reset(float verticalSpeedCmdIn, const KinematicState & state)
{
    pid.unwrapInputs = false;

    // velocity w.r.t. the wind in NED frame
    Eigen::Vector3f velWind_NED_mps = state.q_nw().toRotationMatrix() * state.velocity_Wind_mps();

    float TAS = velWind_NED_mps.norm();

    float velWind_horiz_mps = hypot(velWind_NED_mps(1), velWind_NED_mps(2));

    float gammaCmd = atan2(verticalSpeedCmdIn, velWind_horiz_mps); // positive up
    float gammaMeas = atan2(state.velocity_NED_mps()(2), velWind_horiz_mps);
    pid.reset(gammaCmd, gammaMeas, 0.0f);
}
