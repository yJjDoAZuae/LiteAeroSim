#pragma once

namespace liteaero::simulation {

struct AtmosphericState {
    float temperature_k;          // static (ambient) temperature (K)
    float pressure_pa;            // static pressure (Pa)
    float density_kgm3;           // air density, moist (kg/m³)
    float speed_of_sound_mps;     // (m/s)
    float relative_humidity_nd;   // 0–1, non-dimensional
    float density_altitude_m;     // altitude in ISA where ρ_ISA = ρ_actual (m)
};

} // namespace liteaero::simulation
