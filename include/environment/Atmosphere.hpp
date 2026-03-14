#pragma once
#include "environment/AtmosphericState.hpp"
#include "environment/AtmosphereConfig.hpp"
#include <nlohmann/json.hpp>

namespace liteaerosim::environment {

class Atmosphere {
public:
    explicit Atmosphere(const AtmosphereConfig& config = {});

    // Returns the full atmospheric state at the given geometric altitude (m).
    AtmosphericState state(float altitude_m) const;

    // Convenience accessors.
    float density_kgm3(float altitude_m) const;
    float density_altitude_m(float altitude_m) const;

    // Returns the density ratio sigma = rho(h) / rho(0).
    float density_ratio(float altitude_m) const;

    const AtmosphereConfig& config() const;

    // Serialization.
    nlohmann::json serializeJson() const;
    void deserializeJson(const nlohmann::json& j);

private:
    AtmosphereConfig config_;

    // Pre-computed at construction from config_; never re-computed inside state().
    struct CachedConstants {
        float phi;        // relative humidity (copy of config_.relative_humidity_nd)
        float delta_t_k;  // ISA temperature offset (copy of config_.delta_temperature_k)
    };
    CachedConstants cached_{};

    // ISA layer boundary pressures — compile-time constants, no runtime integration.
    static constexpr float kP_11000m = 22632.1f;   // Pa at 11 000 m geopotential
    static constexpr float kP_20000m =  5474.9f;   // Pa at 20 000 m geopotential

    // Per-query helpers (altitude-dependent only; use cached_ for config values).
    static float isa_pressure_pa(float h_gp_m);
    static float isa_temperature_k(float h_gp_m);
    static float to_geopotential_m(float h_geom_m);
    static float density_altitude_from_density(float density_kgm3);
};

} // namespace liteaerosim::environment
