#pragma once
#include <Eigen/Dense>

namespace liteaero::simulation {

struct TurbulenceVelocity {
    Eigen::Vector3f velocity_body_mps;    // (u_wg, v_wg, w_wg) in body frame (m/s)
    Eigen::Vector3f angular_rate_rad_s;   // (p_wg, q_wg, r_wg) in body frame (rad/s)
};

} // namespace liteaero::simulation
