#include "control/ControlHeadingRate.hpp"
#include <Eigen/Dense>

using namespace Control;

float velWind_horiz(const KinematicState & state)
{
    // velocity w.r.t. the wind in NED frame
    Eigen::Vector3f velWind_NED_mps = state.q_nw().toRotationMatrix() * state.velocity_Wind_mps();

    return hypot(velWind_NED_mps(0), velWind_NED_mps(1));
}

float ControlHeadingRate::step(float headingRateCmdIn, const KinematicState & state)
{
    static constexpr float g = 9.81;

    float ayCmd = velWind_horiz(state)*headingRateCmdIn;

    return ayCmd;
}
