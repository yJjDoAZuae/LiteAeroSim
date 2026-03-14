#include "environment/Turbulence.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace liteaerosim::environment {

// ---------------------------------------------------------------------------
// RngState — defined in this TU only
// ---------------------------------------------------------------------------
struct Turbulence::RngState {
    std::mt19937                    engine;
    std::normal_distribution<float> dist{0.f, 1.f};
};

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------
Turbulence::Turbulence()  = default;
Turbulence::~Turbulence() = default;

// ---------------------------------------------------------------------------
// W20 mapping per MIL-HDBK-1797 §Intensity Levels
// ---------------------------------------------------------------------------
float Turbulence::w20_mps() const {
    switch (config_.intensity) {
        case TurbulenceIntensity::None:     return 0.f;
        case TurbulenceIntensity::Light:    return 3.1f;
        case TurbulenceIntensity::Moderate: return 6.2f;
        case TurbulenceIntensity::Severe:   return 12.4f;
    }
    return 0.f;
}

// ---------------------------------------------------------------------------
// initialize()
// ---------------------------------------------------------------------------
void Turbulence::initialize(const TurbulenceConfig& config) {
    config_ = config;
    if (!rng_) {
        rng_ = std::make_unique<RngState>();
    }
    if (config_.seed == 0) {
        std::random_device rd;
        rng_->engine.seed(rd());
    } else {
        rng_->engine.seed(config_.seed);
    }
    reset();
}

// ---------------------------------------------------------------------------
// reset()
// ---------------------------------------------------------------------------
void Turbulence::reset() {
    state_u_.fill(0.f);
    state_v_.fill(0.f);
    state_w_.fill(0.f);
    state_p_.fill(0.f);
    state_q_.fill(0.f);
    state_r_.fill(0.f);
    last_altitude_m_   = -1.f;
    last_airspeed_mps_ = -1.f;
    prev_w_wg_ = 0.f;
    prev_v_wg_ = 0.f;
}

// ---------------------------------------------------------------------------
// white_noise() — zero-mean, unit-variance Gaussian
// ---------------------------------------------------------------------------
float Turbulence::white_noise() {
    return rng_->dist(rng_->engine);
}

// ---------------------------------------------------------------------------
// scale_lengths_and_intensities()
// Sets L_u, L_v, L_w, sigma_u, sigma_v, sigma_w per MIL-HDBK-1797 App. C
// ---------------------------------------------------------------------------
void Turbulence::scale_lengths_and_intensities(float altitude_m, float /*airspeed_mps*/,
                                                float& L_u, float& L_v, float& L_w,
                                                float& sigma_u, float& sigma_v, float& sigma_w) const {
    float W20 = w20_mps();

    if (altitude_m < 305.f) {
        float h = std::max(altitude_m, 1.f);  // avoid divide-by-zero at h=0
        L_w = h;
        float denom = 0.177f + 0.000823f * h;
        L_u = L_v = h / std::pow(denom, 1.2f);
        sigma_w = 0.1f * W20;
        sigma_u = sigma_v = sigma_w / std::pow(denom, 0.4f);
    } else {
        // High altitude (MIL-HDBK-1797 Table C-I)
        L_u = L_v = L_w = 533.f;
        // Sigma values from MIL-HDBK-1797 Table C-I for each intensity level
        switch (config_.intensity) {
            case TurbulenceIntensity::None:
                sigma_u = sigma_v = sigma_w = 0.f;
                break;
            case TurbulenceIntensity::Light:
                sigma_u = sigma_v = sigma_w = 1.5f;   // m/s
                break;
            case TurbulenceIntensity::Moderate:
                sigma_u = sigma_v = sigma_w = 3.0f;
                break;
            case TurbulenceIntensity::Severe:
                sigma_u = sigma_v = sigma_w = 6.0f;
                break;
        }
    }
}

// ---------------------------------------------------------------------------
// Tustin (bilinear) discretization helpers
// ---------------------------------------------------------------------------

// First-order continuous: H(s) = K / (1 + tau*s)
// Tustin: s -> 2/dt * (z-1)/(z+1)
// Discrete TF: Y(z)/X(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
// b0 = K*dt/(2*tau + dt), b1 = b0, a1 = (dt - 2*tau)/(dt + 2*tau)
static TurbulenceFirstOrderCoeffs tustin_first_order(float K, float tau, float dt) {
    float denom = 2.f * tau + dt;
    float b0 = K * dt / denom;
    float b1 = b0;
    float a1 = (dt - 2.f * tau) / denom;
    return {b0, b1, a1};
}

// Second-order with numerator dynamics:
// H(s) = K * (1 + sqrt(3)*tau*s) / (1 + tau*s)^2
// Let tau1 = tau, use two states.
// After Tustin: derived by polynomial algebra.
// Let c = 2/dt (bilinear constant), tau_c = tau * c.
// Denominator: (1 + tau*s)^2 -> (1 + tau_c*(z-1)/(z+1))^2
//   = ((z+1) + tau_c*(z-1))^2 / (z+1)^2
//   = ((1+tau_c)*z + (1-tau_c))^2 / (z+1)^2
// Numerator: K*(1 + sqrt3*tau*s) -> K*(1 + sqrt3*tau_c*(z-1)/(z+1))
//   = K*((z+1) + sqrt3*tau_c*(z-1)) / (z+1)
//   = K*((1+sqrt3*tau_c)*z + (1-sqrt3*tau_c)) / (z+1)
//
// Overall H(z) = K * [(1+s3tc)*z + (1-s3tc)] * (z+1) / [(1+tc)*z + (1-tc)]^2
//   where s3tc = sqrt3*tau_c, tc = tau_c
// Expanding denominator: [(1+tc)^2*z^2 + 2*(1-tc^2)*z + (1-tc)^2]   (before z^-2 division)
// Numerator (before z^2 division): K*[(1+s3tc)*z^2 + (2-2*s3tc^2+... )] ... compute explicitly.
//
// Direct computation:
static TurbulenceSecondOrderCoeffs tustin_second_order_dryden(float K, float tau, float dt) {
    // bilinear pre-warp not needed for Dryden (gentle roll-off, no pre-warp specified)
    float c   = 2.f / dt;
    float tc  = tau * c;  // tau_c
    float s3  = std::sqrt(3.f);
    float s3tc = s3 * tc;

    // Denominator coefficients (monic z^2 form):
    // D(z) = (1+tc)^2 + 2*(1-tc^2)*z^-1 + (1-tc)^2*z^-2  (after dividing by z^2)
    // But we write in a(1) a(2) form: y + a1*y[n-1] + a2*y[n-2] = b0*x + b1*x[n-1] + b2*x[n-2]
    float D0  = (1.f + tc) * (1.f + tc);
    float D1  = 2.f * (1.f - tc * tc);
    float D2  = (1.f - tc) * (1.f - tc);

    // Numerator: multiply K*(1+s3tc*z^-1 ... let's do it step by step.
    // N(z)/D(z) = K * [(1+s3tc)*z + (1-s3tc)] / [(1+tc)*z + (1-tc)]^2
    //           * (z+1)/1   (extra (z+1) factor from cross multiply in full expansion)
    // Actually: H(z) = K * (1+sqrt3*tau*(z-1)/(z+1)*1/c) / (1 + tau*(z-1)/(z+1)*1/c)^2
    // Re-derive more carefully.
    //
    // Let w = (z-1)/(z+1) (bilinear variable, s = c*w)
    // H = K*(1 + sqrt3*tau*c*w) / (1 + tau*c*w)^2
    // Let alpha = tau*c, beta = sqrt3*tau*c = sqrt3*alpha
    // H(z) = K*(1 + beta*w) / (1 + alpha*w)^2
    // w = (z-1)/(z+1), so 1+alpha*w = (z+1+alpha*z-alpha)/(z+1) = ((1+alpha)*z+(1-alpha))/(z+1)
    // (1+alpha*w)^2 = ((1+alpha)*z+(1-alpha))^2 / (z+1)^2
    // 1+beta*w = ((1+beta)*z+(1-beta))/(z+1)
    // H = K * [(1+beta)*z+(1-beta)] * (z+1) / [(1+alpha)*z+(1-alpha)]^2
    //
    // Expand numerator in powers of z (multiply out):
    // N_z = K * [(1+beta)*z^2 + (1+beta+1-beta)*z + (1-beta)] = K*[(1+beta)*z^2 + 2*z + (1-beta)]
    // Denominator D_z = (1+alpha)^2*z^2 + 2*(1-alpha^2)*z + (1-alpha)^2  (note: (a*z+b)^2 = a^2 z^2 + 2ab z + b^2)
    // Actually: (1+alpha)^2*z^2 + 2*(1+alpha)*(1-alpha)*z + (1-alpha)^2
    //         = (1+alpha)^2*z^2 + 2*(1-alpha^2)*z + (1-alpha)^2
    //
    // Divide everything by D_z[0] = (1+alpha)^2, and convert to z^-1 form:
    float alpha = tc;
    float beta  = s3tc;

    float N0 = K * (1.f + beta);
    float N1 = K * 2.f;
    float N2 = K * (1.f - beta);

    // D2_coef are coefficients of z^2, z^1, z^0 in denominator
    float Da0 = (1.f + alpha) * (1.f + alpha);
    float Da1 = 2.f * (1.f - alpha * alpha);
    float Da2 = (1.f - alpha) * (1.f - alpha);

    // Normalize by Da0:
    return {
        .b0 = N0 / Da0,
        .b1 = N1 / Da0,
        .b2 = N2 / Da0,
        .a1 = Da1 / Da0,
        .a2 = Da2 / Da0,
    };
}

// ---------------------------------------------------------------------------
// recompute_coefficients()
// ---------------------------------------------------------------------------
void Turbulence::recompute_coefficients(float altitude_m, float airspeed_mps) {
    float L_u, L_v, L_w, sigma_u, sigma_v, sigma_w;
    scale_lengths_and_intensities(altitude_m, airspeed_mps, L_u, L_v, L_w, sigma_u, sigma_v, sigma_w);

    float dt = config_.dt_s;
    float Va = std::max(airspeed_mps, 1.f);  // avoid divide-by-zero

    // H_u: first-order.  DC gain K = sigma_u*sqrt(2*tau) ensures
    // output variance = sigma_u^2 for unit-variance (two-sided PSD=1) white noise input.
    // (MIL-HDBK-1797 uses K=sigma*sqrt(2*L/(pi*Va)) with one-sided PSD convention;
    //  we use the equivalent two-sided form K=sigma*sqrt(2*L/Va)=K_MIL*sqrt(pi).)
    {
        float tau = L_u / Va;
        float K   = sigma_u * std::sqrt(2.f * L_u / Va);
        lon_coeffs_ = tustin_first_order(K, tau, dt);
    }

    // H_v: second-order.  K = sigma_v * sqrt(L_v/Va) gives output variance = sigma_v^2.
    {
        float tau = L_v / Va;
        float K   = sigma_v * std::sqrt(L_v / Va);
        lat_coeffs_ = tustin_second_order_dryden(K, tau, dt);
    }

    // H_w: second-order.  K = sigma_w * sqrt(L_w/Va) gives output variance = sigma_w^2.
    {
        float tau = L_w / Va;
        float K   = sigma_w * std::sqrt(L_w / Va);
        vert_coeffs_ = tustin_second_order_dryden(K, tau, dt);
    }

    // H_p: first-order per MIL-HDBK-1797 Appendix C
    // K_p = sigma_w / (Va * sqrt(b_w)) * (pi*b_w / (4*L_w))^(1/6)
    // tau_p = 4*b_w / (pi * Va)  — simplified: tau = 4*b/(pi*Va) ... but document says
    //   H_p(s) = sigma_w/(Va*sqrt(b_w)) * (pi*b_w/(4*L_w))^(1/6) / (1 + (4*b_w/(pi*L_w))*s)
    // The time constant in s is: tau_p = 4*b_w / (pi * L_w / Va)^... let me re-read:
    //   denominator = 1 + (4*b_w / (pi*L_w)) * s   with s in rad/(m) ... actually s here is
    //   temporal (rad/s) since L_w is a spatial scale converted by Va.
    //   tau = 4*b_w / (pi * Va) — checking units: [m] / [m/s] = [s]. Use this.
    {
        float b_w = config_.wingspan_m;
        float K_p = (sigma_w / (Va * std::sqrt(b_w)))
                    * std::pow(static_cast<float>(M_PI) * b_w / (4.f * L_w), 1.f / 6.f);
        float tau_p = 4.f * b_w / (static_cast<float>(M_PI) * Va);
        roll_coeffs_ = tustin_first_order(K_p, tau_p, dt);
    }

    last_altitude_m_   = altitude_m;
    last_airspeed_mps_ = airspeed_mps;
}

// ---------------------------------------------------------------------------
// Direct-form II recurrence helpers
// ---------------------------------------------------------------------------
static float df2_first_order(TurbulenceFirstOrderCoeffs& c,
                              std::array<float, 2>& state, float x) {
    // y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
    // state[0] = x[n-1], state[1] = y[n-1]
    float y = c.b0 * x + c.b1 * state[0] - c.a1 * state[1];
    state[0] = x;
    state[1] = y;
    return y;
}

static float df2_second_order(TurbulenceSecondOrderCoeffs& c,
                               std::array<float, 2>& state, float x) {
    // Direct-form II transposed:
    // y[n] = b0*x[n] + w1[n-1]
    // w1[n] = b1*x[n] - a1*y[n] + w2[n-1]
    // w2[n] = b2*x[n] - a2*y[n]
    float y  = c.b0 * x + state[0];
    float w1 = c.b1 * x - c.a1 * y + state[1];
    float w2 = c.b2 * x - c.a2 * y;
    state[0] = w1;
    state[1] = w2;
    return y;
}

// ---------------------------------------------------------------------------
// step()
// ---------------------------------------------------------------------------
TurbulenceVelocity Turbulence::step(float altitude_m, float airspeed_mps) {
    if (config_.intensity == TurbulenceIntensity::None || !rng_) {
        return {Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
    }

    float dalt = std::abs(altitude_m   - last_altitude_m_);
    float dVa  = std::abs(airspeed_mps - last_airspeed_mps_);
    if (dalt > 1.f || dVa > 0.5f || last_altitude_m_ < 0.f) {
        recompute_coefficients(altitude_m, airspeed_mps);
    }

    float dt = config_.dt_s;
    float Va = std::max(airspeed_mps, 1.f);
    // Scale white noise by 1/sqrt(dt) to preserve PSD
    float inv_sqrt_dt = 1.f / std::sqrt(dt);

    float n_u = white_noise() * inv_sqrt_dt;
    float n_v = white_noise() * inv_sqrt_dt;
    float n_w = white_noise() * inv_sqrt_dt;
    float n_p = white_noise() * inv_sqrt_dt;

    float u_wg = df2_first_order(lon_coeffs_,  state_u_, n_u);
    float v_wg = df2_second_order(lat_coeffs_, state_v_, n_v);
    float w_wg = df2_second_order(vert_coeffs_, state_w_, n_w);
    float p_wg = df2_first_order(roll_coeffs_, state_p_, n_p);

    // q_wg ≈ -(dw_wg/dt) / Va  (pitch rate from vertical velocity gradient)
    // r_wg ≈  (dv_wg/dt) / Va  (yaw rate from lateral velocity gradient)
    float q_wg = -(w_wg - prev_w_wg_) / (dt * Va);
    float r_wg =  (v_wg - prev_v_wg_) / (dt * Va);

    prev_w_wg_ = w_wg;
    prev_v_wg_ = v_wg;

    // Use state arrays for q/r (track previous output for derivative)
    state_q_[0] = w_wg;
    state_r_[0] = v_wg;

    return {
        .velocity_body_mps  = {u_wg, v_wg, w_wg},
        .angular_rate_rad_s = {p_wg, q_wg, r_wg},
    };
}

const TurbulenceConfig& Turbulence::config() const {
    return config_;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------
nlohmann::json Turbulence::serializeJson() const {
    return {
        {"schema_version", config_.schema_version},
        {"intensity",      static_cast<int>(config_.intensity)},
        {"wingspan_m",     config_.wingspan_m},
        {"dt_s",           config_.dt_s},
        {"seed",           config_.seed},
        {"state_u",        std::vector<float>(state_u_.begin(), state_u_.end())},
        {"state_v",        std::vector<float>(state_v_.begin(), state_v_.end())},
        {"state_w",        std::vector<float>(state_w_.begin(), state_w_.end())},
        {"state_p",        std::vector<float>(state_p_.begin(), state_p_.end())},
        {"state_q",        std::vector<float>(state_q_.begin(), state_q_.end())},
        {"state_r",        std::vector<float>(state_r_.begin(), state_r_.end())},
    };
}

void Turbulence::deserializeJson(const nlohmann::json& j) {
    int sv = j.at("schema_version").get<int>();
    if (sv != 1) {
        throw std::runtime_error("Turbulence: unsupported schema_version " + std::to_string(sv));
    }
    TurbulenceConfig cfg;
    cfg.schema_version = sv;
    cfg.intensity      = static_cast<TurbulenceIntensity>(j.at("intensity").get<int>());
    cfg.wingspan_m     = j.at("wingspan_m").get<float>();
    cfg.dt_s           = j.at("dt_s").get<float>();
    cfg.seed           = j.at("seed").get<uint32_t>();

    initialize(cfg);

    auto load_state = [&](const std::string& key, std::array<float, 2>& arr) {
        auto v = j.at(key).get<std::vector<float>>();
        arr[0] = v[0];
        arr[1] = v[1];
    };
    load_state("state_u", state_u_);
    load_state("state_v", state_v_);
    load_state("state_w", state_w_);
    load_state("state_p", state_p_);
    load_state("state_q", state_q_);
    load_state("state_r", state_r_);
}

} // namespace liteaerosim::environment
