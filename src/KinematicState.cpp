
#include "KinematicState.hpp"

KinematicState::KinematicState() 
    : time_sec(0.0),
      velocity_NED_mps(Eigen::Vector3f::Zero()),
      acceleration_NED_mps(Eigen::Vector3f::Zero()),
      q_nb(Eigen::Quaternionf::Identity()),
      bodyRates_rps(Eigen::Vector3f::Zero())
{}

double KinematicState::latitude_rate_rps() {
    // Placeholder implementation
    return 0.0;
}

double KinematicState::longitude_rate_rps() {
    // Placeholder implementation
    return 0.0;
}

Eigen::Vector3f KinematicState::velocity_Wind_mps() {
    // Placeholder implementation
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KinematicState::velocity_Body_mps() {
    // Placeholder implementation
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KinematicState::acceleration_Wind_mps() {
    // Placeholder implementation
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KinematicState::acceleration_Body_mps() {
    // Placeholder implementation
    return Eigen::Vector3f::Zero();
}
