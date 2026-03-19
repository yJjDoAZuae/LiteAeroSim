#include "control/ControlHeading.hpp"

using namespace liteaerosim::control;

float ControlHeading::step(float command, const KinematicState& state)
{
    controller_.setUnwrapInputs(true);
    return controller_.step(command, state.heading(), state.headingRate_rps());
}

void ControlHeading::reset(float command, const KinematicState& state)
{
    controller_.setUnwrapInputs(true);
    controller_.reset(command, state.heading(), 0.0f);
}
