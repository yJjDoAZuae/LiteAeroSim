#include "landing_gear/WheelUnit.hpp"
#include "liteaerosim.pb.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------
// Pacejka magic formula: F(s) = D * sin(C * atan(Bs - E*(Bs - atan(Bs))))
// ---------------------------------------------------------------------------
static float pacejka(float s, float B, float C, float D, float E) {
    const float Bs      = B * s;
    const float atan_Bs = std::atan(Bs);
    return D * std::sin(C * std::atan(Bs - E * (Bs - atan_Bs)));
}

// ---------------------------------------------------------------------------

void WheelUnit::initialize(const WheelUnitParams& params) {
    _params                    = params;
    _strut_deflection_m        = 0.0f;
    _strut_deflection_rate_mps = 0.0f;
    _wheel_speed_rps           = 0.0f;
}

void WheelUnit::reset() {
    _strut_deflection_m        = 0.0f;
    _strut_deflection_rate_mps = 0.0f;
    _wheel_speed_rps           = 0.0f;
}

// ---------------------------------------------------------------------------

WheelContactForces WheelUnit::step(float                         penetration_m,
                                    const Eigen::Vector3f&        contact_point_body_m,
                                    const Eigen::Vector3f&        contact_vel_body_mps,
                                    const Eigen::Vector3f&        surface_normal_body,
                                    float                         wheel_angle_rad,
                                    float                         brake_demand_nd,
                                    float                         friction_mu_nd,
                                    float                         dt_s) {
    if (penetration_m <= 0.0f) {
        _strut_deflection_rate_mps = 0.0f;
        return {};
    }

    // 1. Quasi-static strut deflection
    const float delta_new = std::clamp(penetration_m, 0.0f, _params.travel_max_m);
    const float delta_dot = (dt_s > 0.0f)
                                ? (delta_new - _strut_deflection_m) / dt_s
                                : 0.0f;
    _strut_deflection_rate_mps = delta_dot;
    _strut_deflection_m        = delta_new;

    // 2. Normal (strut) force: F_z = k*delta + preload + b*delta_dot, floored at zero
    const float F_z = std::max(0.0f,
        _params.spring_stiffness_npm * delta_new
        + _params.preload_n
        + _params.damper_coeff_nspm * delta_dot);

    // 3. Wheel heading in body frame
    //    Forward = body-x projected onto the ground plane, then rotated by steering angle.
    Eigen::Vector3f wheel_fwd{1.0f, 0.0f, 0.0f};
    wheel_fwd -= wheel_fwd.dot(surface_normal_body) * surface_normal_body;
    if (wheel_fwd.squaredNorm() < 1e-6f) {
        // Degenerate (aircraft near-vertical): fall back to body-y projected
        Eigen::Vector3f body_y{0.0f, 1.0f, 0.0f};
        wheel_fwd = body_y - body_y.dot(surface_normal_body) * surface_normal_body;
    }
    wheel_fwd.normalize();

    if (_params.is_steerable && std::abs(wheel_angle_rad) > 1e-6f) {
        const Eigen::AngleAxisf steer{wheel_angle_rad, surface_normal_body};
        wheel_fwd = steer * wheel_fwd;
    }

    // Right = forward × normal  (right-hand: fwd × up = right)
    const Eigen::Vector3f wheel_right = wheel_fwd.cross(surface_normal_body).normalized();

    // 4. Contact-patch velocity components
    constexpr float kVeps = 0.01f;  // m/s regularization
    const float V_cx = contact_vel_body_mps.dot(wheel_fwd);
    const float V_cy = contact_vel_body_mps.dot(wheel_right);

    // 5. Slip ratio (longitudinal)
    const float kappa = (_params.tyre_radius_m * _wheel_speed_rps - V_cx)
                        / (std::abs(V_cx) + kVeps);

    // 6. Slip angle (lateral)
    const float alpha_t = -std::atan2(V_cy, std::abs(V_cx) + kVeps);

    // 7. Pacejka tyre forces  (B, C, D=mu*Fz, E from arch doc Table 3d)
    float F_x = pacejka(kappa,   10.0f, 1.9f, friction_mu_nd * F_z,  0.97f);
    float F_y = pacejka(alpha_t,  8.0f, 1.3f, friction_mu_nd * F_z, -1.0f);

    // 8. Friction-circle saturation
    const float F_total = std::sqrt(F_x * F_x + F_y * F_y);
    const float F_limit = friction_mu_nd * F_z;
    if (F_total > F_limit && F_total > 0.0f) {
        const float scale = F_limit / F_total;
        F_x *= scale;
        F_y *= scale;
    }

    // 9. Wheel speed integration (Euler)
    const float r_w           = _params.tyre_radius_m;
    const float wheel_mass_kg = 0.3f * r_w;          // empirical: m_w ≈ 0.3 * r_w
    const float I_w           = wheel_mass_kg * r_w * r_w * 0.5f;

    // Brake torque: τ_brake = C_brake * b * ω_w  (arch doc §4)
    const float tau_brake = (_params.has_brake)
                                ? _params.max_brake_torque_nm * brake_demand_nd * _wheel_speed_rps
                                : 0.0f;

    // Rolling resistance torque with 0.01 rad/s deadband
    const float omega_sign = (_wheel_speed_rps >  0.01f) ?  1.0f
                           : (_wheel_speed_rps < -0.01f) ? -1.0f
                           :                               0.0f;
    const float tau_roll = _params.rolling_resistance_nd * r_w * F_z * omega_sign;

    if (I_w > 0.0f) {
        const float omega_dot = (-r_w * F_x - tau_brake - tau_roll) / I_w;
        _wheel_speed_rps += omega_dot * dt_s;
    }

    // 10. Force assembly in body frame
    const Eigen::Vector3f force = F_z * surface_normal_body
                                 + F_x * wheel_fwd
                                 + F_y * wheel_right;
    WheelContactForces result;
    result.force_body_n   = force;
    result.moment_body_nm = contact_point_body_m.cross(force);
    result.in_contact     = true;
    return result;
}

// ---------------------------------------------------------------------------

StrutState WheelUnit::strutState() const {
    return {_strut_deflection_m, _strut_deflection_rate_mps, _wheel_speed_rps};
}

void WheelUnit::setStrutState(const StrutState& s) {
    _strut_deflection_m        = s.strut_deflection_m;
    _strut_deflection_rate_mps = s.strut_deflection_rate_mps;
    _wheel_speed_rps           = s.wheel_speed_rps;
}

// ---------------------------------------------------------------------------

nlohmann::json WheelUnit::serializeJson() const {
    return {
        {"strut_deflection_m",        _strut_deflection_m},
        {"strut_deflection_rate_mps", _strut_deflection_rate_mps},
        {"wheel_speed_rps",           _wheel_speed_rps},
    };
}

void WheelUnit::deserializeJson(const nlohmann::json& j) {
    _strut_deflection_m        = j.at("strut_deflection_m").get<float>();
    _strut_deflection_rate_mps = j.at("strut_deflection_rate_mps").get<float>();
    _wheel_speed_rps           = j.at("wheel_speed_rps").get<float>();
}

std::vector<uint8_t> WheelUnit::serializeProto() const {
    las_proto::WheelUnitState proto;
    proto.set_strut_deflection_m(_strut_deflection_m);
    proto.set_strut_deflection_rate_mps(_strut_deflection_rate_mps);
    proto.set_wheel_speed_rps(_wheel_speed_rps);
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void WheelUnit::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::WheelUnitState proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())))
        throw std::runtime_error("WheelUnit::deserializeProto: failed to parse");
    _strut_deflection_m        = proto.strut_deflection_m();
    _strut_deflection_rate_mps = proto.strut_deflection_rate_mps();
    _wheel_speed_rps           = proto.wheel_speed_rps();
}

}  // namespace liteaero::simulation
