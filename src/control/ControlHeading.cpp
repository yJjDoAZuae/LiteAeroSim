#include "control/ControlHeading.hpp"

using namespace Control;

float ControlHeading::step(float cmdIn, const KinematicState & state)
{
    pid.unwrapInputs = true;
    return pid.step(cmdIn, state.heading());
}

void ControlHeading::reset(float cmdIn, const KinematicState & state)
{
    pid.unwrapInputs = true;
    pid.reset(cmdIn, state.heading(), 0.0f);
}
