#include "control/ControlLoadFactor.hpp"
#include <Eigen/Dense>

using namespace liteaerosim::control;

static float constexpr g = 9.81;

float ControlLoadFactor::step(float load_factor_command, const KinematicState& state)
{
    return controller_.step(load_factor_command, -state.acceleration_Wind_mps()(2) / g);
}

void ControlLoadFactor::reset(float load_factor_command, const KinematicState& state)
{
    controller_.reset(load_factor_command, -state.acceleration_Wind_mps()(2) / g, 0.0f);
    controller_.integrator().resetTo(0.0f);
}
