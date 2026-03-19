#include "control/ControlRoll.hpp"

using namespace liteaerosim::control;

float ControlRoll::step(float command, const KinematicState& state)
{
    controller_.setUnwrapInputs(true);

    // get attitude Eulers of Wind frame
    Eigen::Vector3f eulersWind = state.q_nw().toRotationMatrix().eulerAngles(3,2,1);

    // get Body rates expressed in wind axes
    Eigen::Vector3f ratesWind = state.q_nw().toRotationMatrix().transpose() * state.q_nb().toRotationMatrix() * state.rates_Body_rps();

    // NOTE: roll and rollRate_rps aren't correct for velocity roll control
    // use roll Euler of Wind frame w.r.t. NED and the roll rate of the Wind frame
    return controller_.step(command, eulersWind(0), state.rollRate_Wind_rps());
}

void ControlRoll::reset(float command, const KinematicState& state)
{
    controller_.setUnwrapInputs(true);

    // get attitude Eulers of Wind frame
    Eigen::Vector3f eulersWind = state.q_nw().toRotationMatrix().eulerAngles(3,2,1);

    controller_.reset(command, eulersWind(0), 0.0f);
}
