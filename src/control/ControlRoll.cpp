#include "control/ControlRoll.hpp"

using namespace Control;

float ControlRoll::step(float cmdIn, const KinematicState & state)
{
    pid.unwrapInputs = true;
    return pid.step(cmdIn, state.roll(), state.rollRate_rps());
}

void ControlRoll::reset(float cmdIn, const KinematicState & state)
{
    pid.unwrapInputs = true;
    pid.reset(cmdIn, state.roll(), 0.0f);
}
