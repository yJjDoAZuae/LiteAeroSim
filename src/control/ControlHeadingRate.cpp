#include "control/ControlHeadingRate.hpp"

using namespace Control;

float ControlHeadingRate::step(float headingRateCmdIn, const KinematicState & state)
{
    static constexpr float g = 9.81;

    float TAS = state.velocity_Wind_mps().norm();
    float ayCmd = TAS*headingRateCmdIn;

    float phiCmd = atan(ayCmd/g);

    return phiCmd;
}
