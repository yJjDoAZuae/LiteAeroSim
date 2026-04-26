#pragma once
#include "environment/AtmosphericState.hpp"
#include "environment/AtmosphereConfig.hpp"
#include <nlohmann/json.hpp>

namespace liteaero::geodesy { class Egm2008Geoid; }

namespace liteaero::simulation {

class Atmosphere {
public:
    explicit Atmosphere(const AtmosphereConfig& config = {});

    // Returns the full atmospheric state at the given altitude.
    //
    // INPUT CONVENTION: altitude_m is **mean sea level (MSL) geopotential
    // altitude** — the standard ISA reference.  Callers holding WGS84
    // ellipsoidal heights must convert via the geoid undulation (use the
    // state_at_wgs84_m / density_at_wgs84_m helpers below) so the atmosphere
    // model is fed the geometric quantity it expects.  See OQ-LS-14 in
    // docs/architecture/live_sim_view.md.
    AtmosphericState state(float altitude_msl_m) const;

    // Convenience accessors.  Same MSL convention as state().
    float density_kgm3(float altitude_msl_m) const;
    float density_altitude_m(float altitude_msl_m) const;

    // Returns the density ratio sigma = rho(h) / rho(0).
    float density_ratio(float altitude_msl_m) const;

    // ------------------------------------------------------------------
    // WGS84-ellipsoidal-input overloads (LS-T6 / OQ-LS-14)
    //
    // These accept the aircraft's WGS84 ellipsoidal height directly and
    // perform the WGS84 -> MSL conversion via the supplied Egm2008Geoid.
    // When `geoid` is null, the call falls back to treating h_wgs84_m as
    // MSL with no correction — the caller has accepted the (~ N) bias.
    // ------------------------------------------------------------------
    AtmosphericState state_at_wgs84_m(
        double lat_rad, double lon_rad, float h_wgs84_m,
        const liteaero::geodesy::Egm2008Geoid* geoid = nullptr) const;

    float density_at_wgs84_m(
        double lat_rad, double lon_rad, float h_wgs84_m,
        const liteaero::geodesy::Egm2008Geoid* geoid = nullptr) const;

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

} // namespace liteaero::simulation
