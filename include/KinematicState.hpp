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
                       _q_nb(Eigen::Quaternionf::Identity()),
                       _rates_Body_rps(Eigen::Vector3f::Zero()) {};

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
          _q_nb(Eigen::Quaternionf::Identity()),
          _rates_Body_rps(Eigen::Vector3f::Zero())
    {

        const float smallV(0.1);

        Eigen::Quaternionf local_q_nv = this->q_nv();
        stepQnv(_velocity_NED_mps, local_q_nv);

        // alpha, beta -> q_wb
        Eigen::Quaternionf q_wb(Eigen::AngleAxisf(alpha, Eigen::Vector3f(0,1,0)) * Eigen::AngleAxisf(beta, Eigen::Vector3f(0,0,1)));

        _q_nb = q_nw*q_wb;

        // TODO: alphaDot, betaDot -> omega_bw_w
        Eigen::Vector3f omega_bw_w;  // rotation rate of Body w.r.t. Wind expressed in the Wind frame
        // TODO: implement Euler rate to body rate matrix

        // accel, vel, -> omega_wn_n
        // this is path curvature induced rotation in the POM
        Eigen::Vector3f omega_wn_n(Eigen::Vector3f::Zero());  // rotation rate of Wind w.r.t. NED expressed in the NED frame

        float normV = _velocity_NED_mps.norm();
        if (normV > smallV) {
            // omega = V/R = kappa*V
            // ay = V^2/R = V^2 * kappa = V*omega

            // path curvature
            Eigen::Vector3f kappa(_velocity_NED_mps.cross(_acceleration_NED_mps)/(normV*normV*normV));

            omega_wn_n = kappa*normV;
        }

        // sum angular rate contributions
        _rates_Body_rps = q_wb.toRotationMatrix().transpose() * (q_nw.toRotationMatrix().transpose()*omega_wn_n + omega_bw_w);
    }

    KinematicState(double time_sec,
                   const WGS84_Datum &position_datum,
                   const Eigen::Vector3f &velocity_NED_mps,
                   const Eigen::Vector3f &acceleration_NED_mps,
                   const Eigen::Quaternionf &q_nb,
                   const Eigen::Vector3f &rates_Body_rps) 
        : _time_sec(time_sec),
          _positionDatum(position_datum),
          _velocity_NED_mps(velocity_NED_mps),
          _acceleration_NED_mps(acceleration_NED_mps),
          _q_nb(q_nb),
          _rates_Body_rps(rates_Body_rps)
          {}

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
    Eigen::Quaternionf q_nb() const { return _q_nb; }   // Body to NED rotation
    Eigen::Vector3f rates_Body_rps() const { return _rates_Body_rps; }

    // derived quantity methods
    double latitudeRate_rps() const;
    double longitudeRate_rps() const;
    Eigen::Vector3f velocity_Wind_mps() const;
    Eigen::Vector3f velocity_Stab_mps() const;
    Eigen::Vector3f velocity_Body_mps() const;
    Eigen::Vector3f acceleration_Body_mps() const;
    Eigen::Vector3f acceleration_Wind_mps() const;
    Eigen::Vector3f eulers() const;
    float roll() const;
    float pitch() const;
    float heading() const;
    float rollRate_rps() const; // roll Euler time derivative
    float pitchRate_rps() const; // pitch Euler time derivative
    float headingRate_rps() const; // heading Euler time derivative

    PlaneOfMotion &POM() const;
    TurnCircle &turnCircle() const;

    Eigen::Quaternionf q_nl() const; // Local Level to NED rotation
    Eigen::Quaternionf q_ns() const; // Stability to NED rotation
    Eigen::Quaternionf q_nw() const; // Wind to NED rotation
    Eigen::Quaternionf q_nv() const; // Velocity to NED rotation
    float alpha() const;
    float beta() const;
    float rollRate_Wind_rps() const;
    float alphaDot() const;
    float betaDot() const;

protected:

    // time
    double _time_sec;

    // position
    WGS84_Datum _positionDatum;

    // velocity
    Eigen::Vector3f _velocity_NED_mps;

    // acceleration in NED frame
    Eigen::Vector3f _acceleration_NED_mps;

    // Body to NED rotation
    Eigen::Quaternionf _q_nb;

    // Body rates
    Eigen::Vector3f _rates_Body_rps;

    static void stepQnv(const Eigen::Vector3f& velocity_NED_mps, Eigen::Quaternionf& q_nv );

};
