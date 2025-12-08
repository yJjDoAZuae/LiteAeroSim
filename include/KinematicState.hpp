#pragma once

// Kinematic/temporal state and all quantities derived from kinematics

// Jerk and angular accelerations are not included, so trajectory torsion will not be calculated

#include "navigation/WGS84.hpp"
#include <Eigen/Dense>

class PlaneOfMotion
{

public:
    Eigen::Quaternionf q_np; // POM to NED rotation
};

class TurnCircle
{

public:
    PlaneOfMotion pom;

    Eigen::Vector3f turnCenter_deltaNED_m;
};

class KinematicState
{

public:
    KinematicState() : _time_sec(0.0),
                       _positionDatum(),
                       _velocity_NED_mps(Eigen::Vector3f::Zero()),
                       _acceleration_NED_mps(Eigen::Vector3f::Zero()),
                       _q_nw(Eigen::Quaternionf::Identity()),
                       _alpha(0.0f),
                       _beta(0.0f),
                       _alphaDot(0.0f),
                       _betaDot(0.0f) {};

    KinematicState(double time_sec,
                   const WGS84_Datum &position_datum,
                   const Eigen::Vector3f &velocity_NED_mps,
                   const Eigen::Vector3f &acceleration_Wind_mps,
                   const Eigen::Quaternionf &q_nw,
                   float alpha,
                   float beta,
                   float alphaDot,
                   float betaDot)
        : _time_sec(time_sec),
          _positionDatum(position_datum),
          _velocity_NED_mps(velocity_NED_mps),
          _acceleration_NED_mps(q_nw.toRotationMatrix()*acceleration_Wind_mps),
          _q_nw(q_nw),
          _alpha(alpha),
          _beta(beta),
          _alphaDot(alphaDot),
          _betaDot(betaDot) { }

    KinematicState(double time_sec,
                   const WGS84_Datum &position_datum,
                   const Eigen::Vector3f &velocity_NED_mps,
                   const Eigen::Vector3f &acceleration_NED_mps,
                   const Eigen::Quaternionf &q_nb,
                   const Eigen::Vector3f &rates_Body_rps);

    ~KinematicState() {};

    // state update
    void step(double time_sec,
              Eigen::Vector3f acceleration_Wind_mps,
              float rollRate_Wind_rps,
              float alpha,
              float beta,
              float alphaDot,
              float betaDot);

    // getters
    double time_sec() const { return _time_sec; }
    WGS84_Datum positionDatum() const { return _positionDatum; }
    Eigen::Vector3f velocity_NED_mps() const { return _velocity_NED_mps; }
    Eigen::Vector3f acceleration_NED_mps() const { return _acceleration_NED_mps; }
    Eigen::Quaternionf q_nw() const { return _q_nw; }  // Wind to NED rotation
    Eigen::Quaternionf q_nv() const { return _q_nv; }  // Velocity to NED rotation
    float alpha() const { return _alpha; }
    float beta() const { return _beta; }
    float rollRate_Wind_rps() const { return _rollRate_Wind_rps; }
    float alphaDot() const { return _alphaDot; }
    float betaDot() const { return _betaDot; }

    // derived quantity methods
    double latitude_rate_rps() const;
    double longitude_rate_rps() const;
    Eigen::Vector3f velocity_Wind_mps() const;
    Eigen::Vector3f velocity_Stab_mps() const;
    Eigen::Vector3f velocity_Body_mps() const;
    Eigen::Vector3f acceleration_Body_mps() const;
    Eigen::Vector3f acceleration_Wind_mps() const;
    Eigen::Vector3f eulers() const;
    float roll() const;
    float pitch() const;
    float heading() const;
    Eigen::Vector3f bodyRates_rps() const;

    PlaneOfMotion &POM() const;
    TurnCircle &turnCircle() const;

    Eigen::Quaternionf q_nl() const;                   // Local Level to NED rotation
    Eigen::Quaternionf q_ns() const;                   // Stability to NED rotation
    Eigen::Quaternionf q_nb() const;                   // Body to NED rotation

protected:

    // time
    double _time_sec;

    // position
    WGS84_Datum _positionDatum;

    // velocity
    Eigen::Vector3f _velocity_NED_mps;

    // acceleration in NED frame
    Eigen::Vector3f _acceleration_NED_mps;

    // roll rate about the velocity vector
    float _rollRate_Wind_rps;

    // wind to NED rotation
    Eigen::Quaternionf _q_nw;

    // velocity to NED rotation
    Eigen::Quaternionf _q_nv;

    // aerodynamic angles
    // TODO: should alpha and beta be stateful and integrate their rates?
    // Should we filter to enforce consistency?
    float _alpha; // angle of attack, rad
    float _beta;  // sideslip angle, rad

    // aerodynamic angle rates
    float _alphaDot; // angle of attack time derivative, rad/sec
    float _betaDot;  // sideslip angle time derivative, rad/sec

};
