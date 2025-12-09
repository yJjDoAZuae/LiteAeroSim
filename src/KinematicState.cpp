
#include "KinematicState.hpp"

KinematicState::KinematicState(double time_sec,
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
      _acceleration_NED_mps(q_nw.toRotationMatrix() * acceleration_Wind_mps),
      _q_nb(Eigen::Quaternionf::Identity()),
      _rates_Body_rps(Eigen::Vector3f::Zero())
{
    const float smallV(0.1);

    Eigen::Quaternionf local_q_nv = this->q_nv();
    stepQnv(_velocity_NED_mps, local_q_nv);

    // alpha, beta -> q_wb
    Eigen::Quaternionf q_wb(Eigen::AngleAxisf(alpha, Eigen::Vector3f(0, 1, 0)) * Eigen::AngleAxisf(-beta, Eigen::Vector3f(0, 0, 1)));

    _q_nb = q_nw * q_wb;

    // alphaDot, betaDot -> omega_bw_b
    Eigen::Vector3f omega_bw_b; // rotation rate of Body w.r.t. Wind expressed in the Body frame
    omega_bw_b << sin(alpha)*betaDot, alphaDot, -cos(alpha)*betaDot;

    // accel, vel, -> omega_wn_n
    // this is path curvature induced rotation in the POM
    Eigen::Vector3f omega_wn_n(Eigen::Vector3f::Zero()); // rotation rate of Wind w.r.t. NED expressed in the NED frame

    float normV = _velocity_NED_mps.norm();
    if (normV > smallV)
    {
        // omega = V/R = kappa*V
        // ay = V^2/R = V^2 * kappa = V*omega

        // path curvature
        Eigen::Vector3f kappa(_velocity_NED_mps.cross(_acceleration_NED_mps) / (normV * normV * normV));

        omega_wn_n = kappa * normV;
    }

    // sum angular rate contributions
    _rates_Body_rps = omega_bw_b + q_wb.toRotationMatrix().transpose() * (q_nw.toRotationMatrix().transpose() * omega_wn_n);
}

double KinematicState::latitudeRate_rps() const
{
    // Placeholder implementation
    return 0.0;
}

double KinematicState::longitudeRate_rps() const
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
    return q_nb().toRotationMatrix().eulerAngles(3,2,1);
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

Eigen::Quaternionf KinematicState::q_nv() const
{
    return Eigen::Quaternionf::Identity();
}


// update the velocity frame based on the velocity vector
// velocity frame has its 
// first axis aligned with the velocity vector
// second axis right perp in the local level frame
// third axis to complete the triad
void KinematicState::stepQnv(const Eigen::Vector3f& velocity_NED_mps, Eigen::Quaternionf& q_nv )
{
    const float tol = 1e-6;

    Eigen::Vector3f T_hat = velocity_NED_mps.normalized();

    Eigen::Vector3f k_hat(0,0,1); // NED Down vector

    Eigen::Vector3f y_hat = k_hat.cross(T_hat);

    Eigen::Matrix3f CNV;

    if (y_hat.norm() < tol) {
        // if velocity is nearly straight up or down, then
        // make the wind axis azimuth irrotational
        // with respect to the previous orientation of the velocity frame
        // to avoid unnecessary discontinuity of the rotation
        CNV = q_nv.toRotationMatrix();

        y_hat = CNV.col(1);
        y_hat(2) = 0.0f;
        y_hat.normalize();
    } else {
        y_hat.normalize();
    }

    Eigen::Vector3f z_hat = T_hat.cross(y_hat).normalized();

    CNV << T_hat,y_hat,z_hat;

    q_nv = Eigen::Quaternionf(CNV);
}

void KinematicState::step(double time_sec,
              Eigen::Vector3f acceleration_Wind_mps,
              float rollRate_Wind_rps,
              float alpha,
              float beta,
              float alphaDot,
              float betaDot)
{

    // update time
    float dt = float(time_sec - _time_sec);
    _time_sec = time_sec;

    // get accelerations in the NED frame
    Eigen::Vector3f accel_NED = q_nw().toRotationMatrix()*acceleration_Wind_mps;

    // update the velocity vector based on acceleration
    Eigen::Vector3f velocity_NED_mps_prev = _velocity_NED_mps; // we need to save this to determine the rotations
    _velocity_NED_mps += 0.5*(_acceleration_NED_mps + accel_NED)*dt;
    _acceleration_NED_mps = accel_NED;

    // update the velocity frame to align with the new velocity vector
    Eigen::Quaternionf local_q_nv = this->q_nv();
    stepQnv(_velocity_NED_mps, local_q_nv);

    // TODO: update the Wind frame based on acceleration-induced 
    // velocity vector rotation in Wind Y and Z axes

    // TODO: rotate the Wind frame based on rollRate_Wind_rps in
    // Wind X axis

    // TODO: update Body frame based on alpha and beta

    // TODO: get angular rates based on acceleration, rollRate_Wind_rps, alphaDot, and betaDot

}

// https://stengel.mycpanel.princeton.edu/Quaternions.pdf
Eigen::Vector3f KinematicState::EulerRatesToBodyRates(const EulerAngles& ang, const EulerRates& ratesEuler)
{
    float sphi = sin(ang(0));
    float cphi = cos(ang(0));
    float stht = sin(ang(1));
    float ctht = cos(ang(1));

    Eigen::Matrix3f C;
    C << 1, 0, -stht, 0, cphi, sphi*ctht, 0, -sphi, cphi*ctht;

    return C*ratesEuler;
}

// https://stengel.mycpanel.princeton.edu/Quaternions.pdf
EulerRates KinematicState::BodyRatesToEulerRates(const EulerAngles& ang, const Eigen::Vector3f& ratesBody)
{
    const float tol = 1e-6;

    float sphi = sin(ang(0));
    float cphi = cos(ang(0));
    float stht = sin(ang(1));
    float ctht = cos(ang(1));

    // check for gimbal lock
    if (fabs(ctht) > tol) {
        Eigen::Matrix3f C;
        C << 1, sphi*stht/ctht, cphi*stht/ctht, 0, cphi, -sphi, 0, sphi/ctht, cphi/ctht;
        return EulerRates(C*ratesBody);
    }

    return EulerRates::Zero();
}
