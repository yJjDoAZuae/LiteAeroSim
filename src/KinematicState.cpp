
#include "KinematicState.hpp"

KinematicState::KinematicState() 
    : time_sec(0.0),
      velocity_NED_mps(Eigen::Vector3f::Zero()),
      acceleration_NED_mps(Eigen::Vector3f::Zero()),
      q_nb(Eigen::Quaternionf::Identity()),
      bodyRates_rps(Eigen::Vector3f::Zero())
{}

double KinematicState::latitude_rate_rps() const
{
    // Placeholder implementation
    return 0.0;
}

double KinematicState::longitude_rate_rps() const
{
    // Placeholder implementation
    return 0.0;
}

Eigen::Vector3f KinematicState::velocity_Wind_mps() const
{
    // Placeholder implementation
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KinematicState::velocity_Body_mps() const
{
    // Placeholder implementation
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KinematicState::acceleration_Wind_mps() const
{
    // Placeholder implementation
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KinematicState::acceleration_Body_mps() const 
{
    // Placeholder implementation
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KinematicState::eulers() const 
{
    return q_nb.toRotationMatrix().eulerAngles(3,2,1);
}

float KinematicState::roll() const
{
    Eigen::Vector3f eulers = this->eulers();
    return eulers(0);
}

float KinematicState::pitch() const
{
    Eigen::Vector3f eulers = this->eulers();
    return eulers(1);
}

float KinematicState::heading() const
{
    Eigen::Vector3f eulers = this->eulers();
    return eulers(2);
}

