#include "control/ControlAltitude.hpp"

using namespace liteaerosim::control;

float ControlAltitude::step(float command, const KinematicState& state)
{
    controller_.setUnwrapInputs(true);
    return controller_.step(command, state.positionDatum().height_WGS84_m(), -state.velocity_NED_mps()(2));
}

void ControlAltitude::reset(float command, const KinematicState& state)
{
    controller_.setUnwrapInputs(true);
    controller_.reset(command, state.positionDatum().height_WGS84_m(), 0.0f);
}
