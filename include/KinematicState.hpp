// Kinematic/temporal state and all quantities derived from kinematics

// Jerk and angular accelerations are not included, so trajectory torsion will not be calculated

#include <Eigen/Dense>
#include "Navigation/WGS84.hpp"

class PlaneOfMotion {

    Eigen::Quaternionf q_np; // POM to NED rotation
    
};

class TurnCircle {

    PlaneOfMotion pom;

    Eigen::Vector3f turnCenter_deltaNED_m;

};

class KinematicState {

    // time
    double time_sec;

    // position
    WGS84_Datum position_datum;

    // velocity
    Eigen::Vector3f velocity_NED_mps;

    // acceleration
    Eigen::Vector3f acceleration_NED_mps;

    // orientation
    Eigen::Quaternionf q_nb;  // Body to NED rotation

    // orientation rates
    Eigen::Vector3f bodyRates_rps;

    // derived quantity getters
    double latitude_rate_rps();
    double longitude_rate_rps();
    Eigen::Vector3f velocity_Wind_mps();
    Eigen::Vector3f velocity_Body_mps();
    Eigen::Vector3f acceleration_Wind_mps();
    Eigen::Vector3f acceleration_Body_mps();

    KinematicState();
    ~KinematicState() {};

};
