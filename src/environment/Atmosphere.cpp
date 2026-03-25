#include "environment/Atmosphere.hpp"
#include <cmath>
#include <stdexcept>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------
static constexpr float kRe        = 6356766.f;     // WGS-84 mean Earth radius (m)
static constexpr float kG0        = 9.80665f;       // standard gravity (m/s²)
static constexpr float kRd        = 287.058f;       // dry air gas constant (J/(kg·K))
static constexpr float kRv        = 461.495f;       // water vapor gas constant (J/(kg·K))
static constexpr float kEps       = kRd / kRv;      // ≈ 0.6219
static constexpr float kGamma     = 1.4f;           // ratio of specific heats
static constexpr float kT0        = 288.15f;        // ISA SL temperature (K)
static constexpr float kP0        = 101325.f;       // ISA SL pressure (Pa)
static constexpr float kRho0      = 1.225f;         // ISA SL density (kg/m³)
static constexpr float kLtrop     = -0.0065f;       // troposphere lapse rate (K/m)
static constexpr float kTtrop     = 216.65f;        // tropopause / lower-strat base temp (K)
static constexpr float kLstrat    = 0.001f;         // lower stratosphere lapse rate (K/m)
static constexpr float kH_trop    = 11000.f;        // troposphere top (geopotential m)
static constexpr float kH_tpause  = 20000.f;        // tropopause top (geopotential m)
static constexpr float kH_strat   = 32000.f;        // lower stratosphere top (geopotential m)

// Exponent for troposphere pressure law: g0 / (Rd * |Ltrop|) ≈ 5.2559
static constexpr float kTropExp   = kG0 / (kRd * (-kLtrop));

// Exponent for density-altitude inversion: 1 / (g0/(Rd*|Ltrop|) - 1) ≈ 1/4.256
static constexpr float kDaInvExp  = 1.f / (kTropExp - 1.f);

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Atmosphere::Atmosphere(const AtmosphereConfig& config)
    : config_(config)
    , cached_{config.relative_humidity_nd, config.delta_temperature_k}
{}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------
float Atmosphere::to_geopotential_m(float h_geom_m) {
    return kRe * h_geom_m / (kRe + h_geom_m);
}

float Atmosphere::isa_temperature_k(float h_gp_m) {
    if (h_gp_m <= kH_trop) {
        return kT0 + kLtrop * h_gp_m;
    } else if (h_gp_m <= kH_tpause) {
        return kTtrop;
    } else {
        float h = std::min(h_gp_m, kH_strat);
        return kTtrop + kLstrat * (h - kH_tpause);
    }
}

float Atmosphere::isa_pressure_pa(float h_gp_m) {
    if (h_gp_m <= kH_trop) {
        float T = kT0 + kLtrop * h_gp_m;
        return kP0 * std::pow(T / kT0, kTropExp);
    } else if (h_gp_m <= kH_tpause) {
        return kP_11000m * std::exp(-kG0 * (h_gp_m - kH_trop) / (kRd * kTtrop));
    } else {
        float h = std::min(h_gp_m, kH_strat);
        float T = kTtrop + kLstrat * (h - kH_tpause);
        float exp = kG0 / (kRd * kLstrat);
        return kP_20000m * std::pow(T / kTtrop, -exp);
    }
}

float Atmosphere::density_altitude_from_density(float rho) {
    // Troposphere: invert rho = rho0 * (T/T0)^(g0/(Rd*Ltrop) - 1)
    // h_d = (T0 / Ltrop) * (1 - (rho/rho0)^(1/(g0/(Rd*|Ltrop|)-1)))
    // Note: Ltrop is negative, so (T0/Ltrop) is negative; solve carefully.
    // Standard form: h_d = (T0/|Ltrop|) * (1 - (rho/rho0)^kDaInvExp)
    float rho_ratio = rho / kRho0;
    float h_d_trop = (kT0 / (-kLtrop)) * (1.f - std::pow(rho_ratio, kDaInvExp));

    // If within troposphere range, use closed form
    if (h_d_trop >= 0.f && h_d_trop <= kH_trop) {
        return h_d_trop;
    }

    // Above tropopause: bisection on isothermal-layer density formula
    // rho(h) = P(h) / (Rd * Ttrop) where P(h) = kP_11000m * exp(-g0*(h-11000)/(Rd*Ttrop))
    // Target: rho_isa(h) = rho
    auto rho_isa_isothermal = [](float h) -> float {
        float P = Atmosphere::kP_11000m * std::exp(-kG0 * (h - kH_trop) / (kRd * kTtrop));
        return P / (kRd * kTtrop);
    };

    // Bisection over [11000, 40000] m
    float lo = kH_trop;
    float hi = 40000.f;
    for (int i = 0; i < 20; ++i) {
        float mid = 0.5f * (lo + hi);
        if (rho_isa_isothermal(mid) > rho) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5f * (lo + hi);
}

// ---------------------------------------------------------------------------
// state()
// ---------------------------------------------------------------------------
AtmosphericState Atmosphere::state(float altitude_m) const {
    float h_gp = to_geopotential_m(altitude_m);
    h_gp = std::max(h_gp, 0.f);
    h_gp = std::min(h_gp, kH_strat);

    float T_isa = isa_temperature_k(h_gp);
    float P     = isa_pressure_pa(h_gp);
    float T     = T_isa + cached_.delta_t_k;

    // Saturation vapor pressure — Buck (1981) equation
    float Tc  = T - 273.15f;
    float e_s = 611.21f * std::exp((18.678f - Tc / 234.5f) * Tc / (257.14f + Tc));
    float e   = cached_.phi * e_s;
    // Specific humidity
    float q   = kEps * e / (P - (1.f - kEps) * e);
    // Virtual temperature
    float T_v = T * (1.f + q / kEps) / (1.f + q);
    // Density of moist air
    float rho = P / (kRd * T_v);
    // Speed of sound
    float c   = std::sqrt(kGamma * kRd * T_v);
    // Density altitude
    float h_d = density_altitude_from_density(rho);

    return {
        .temperature_k      = T,
        .pressure_pa        = P,
        .density_kgm3       = rho,
        .speed_of_sound_mps = c,
        .relative_humidity_nd = cached_.phi,
        .density_altitude_m = h_d,
    };
}

// ---------------------------------------------------------------------------
// Convenience accessors
// ---------------------------------------------------------------------------
float Atmosphere::density_kgm3(float altitude_m) const {
    return state(altitude_m).density_kgm3;
}

float Atmosphere::density_altitude_m(float altitude_m) const {
    return state(altitude_m).density_altitude_m;
}

float Atmosphere::density_ratio(float altitude_m) const {
    // Compute rho(h) / rho(0) to give exactly 1.0 at sea level.
    return density_kgm3(altitude_m) / density_kgm3(0.f);
}

const AtmosphereConfig& Atmosphere::config() const {
    return config_;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------
nlohmann::json Atmosphere::serializeJson() const {
    return {
        {"schema_version",       config_.schema_version},
        {"delta_temperature_k",  config_.delta_temperature_k},
        {"relative_humidity_nd", config_.relative_humidity_nd},
    };
}

void Atmosphere::deserializeJson(const nlohmann::json& j) {
    int sv = j.at("schema_version").get<int>();
    if (sv != 1) {
        throw std::runtime_error("Atmosphere: unsupported schema_version " + std::to_string(sv));
    }
    AtmosphereConfig cfg;
    cfg.schema_version       = sv;
    cfg.delta_temperature_k  = j.at("delta_temperature_k").get<float>();
    cfg.relative_humidity_nd = j.at("relative_humidity_nd").get<float>();
    config_  = cfg;
    cached_  = {cfg.relative_humidity_nd, cfg.delta_temperature_k};
}

} // namespace liteaero::simulation
