#pragma once
#include "environment/TurbulenceVelocity.hpp"
#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>

namespace liteaerosim::environment {

enum class TurbulenceIntensity { None, Light, Moderate, Severe };

// Discretized filter coefficient structs — at namespace scope so that the
// free helper functions in Turbulence.cpp can access them.
struct TurbulenceFirstOrderCoeffs  { float b0, b1, a1; };
struct TurbulenceSecondOrderCoeffs { float b0, b1, b2, a1, a2; };

struct TurbulenceConfig {
    TurbulenceIntensity intensity      = TurbulenceIntensity::None;
    float               wingspan_m     = 3.0f;    // aircraft wingspan (m); used for angular turbulence
    float               dt_s           = 0.01f;   // simulation timestep (s)
    uint32_t            seed           = 0;        // 0 = random seed from system entropy
    int                 schema_version = 1;
};

class Turbulence {
public:
    Turbulence();
    ~Turbulence();  // defined in .cpp where RngState is complete

    void initialize(const TurbulenceConfig& config);
    void reset();

    // Advances turbulence one timestep.
    // altitude_m: geometric altitude (m); airspeed_mps: true airspeed (m/s).
    TurbulenceVelocity step(float altitude_m, float airspeed_mps);

    const TurbulenceConfig& config() const;

    nlohmann::json serializeJson() const;
    void deserializeJson(const nlohmann::json& j);

private:
    TurbulenceConfig config_;

    // Six second-order filter states (u, v, w translational; p, q, r angular).
    // Each section stored as [w1, w2] (direct-form II transposed delay elements).
    std::array<float, 2> state_u_{};
    std::array<float, 2> state_v_{};
    std::array<float, 2> state_w_{};
    std::array<float, 2> state_p_{};
    std::array<float, 2> state_q_{};  // unused — reserved for future use
    std::array<float, 2> state_r_{};  // unused — reserved for future use

    // Cached parameters (recomputed when altitude or airspeed changes significantly).
    float last_altitude_m_   = -1.f;
    float last_airspeed_mps_ = -1.f;

    TurbulenceFirstOrderCoeffs  lon_coeffs_{};   // H_u (longitudinal, first-order)
    TurbulenceSecondOrderCoeffs lat_coeffs_{};   // H_v (lateral, second-order)
    TurbulenceSecondOrderCoeffs vert_coeffs_{};  // H_w (vertical, second-order)
    TurbulenceFirstOrderCoeffs  roll_coeffs_{};  // H_p (roll, first-order)

    // Previous outputs for q/r derivative approximation
    float prev_w_wg_ = 0.f;
    float prev_v_wg_ = 0.f;

    // Random number generation — pimpl to avoid <random> in header
    struct RngState;
    std::unique_ptr<RngState> rng_;

    void recompute_coefficients(float altitude_m, float airspeed_mps);
    void scale_lengths_and_intensities(float altitude_m, float airspeed_mps,
                                       float& L_u, float& L_v, float& L_w,
                                       float& sigma_u, float& sigma_v, float& sigma_w) const;
    float white_noise();
    float w20_mps() const;
};

} // namespace liteaerosim::environment
