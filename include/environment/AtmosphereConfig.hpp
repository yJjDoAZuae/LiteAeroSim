#pragma once

namespace liteaero::simulation {

struct AtmosphereConfig {
    float delta_temperature_k  = 0.f;   // ISA temperature deviation ΔT (K); positive = warm day
    float relative_humidity_nd = 0.f;   // 0 = dry air, 1 = fully saturated
    int   schema_version       = 1;
};

} // namespace liteaero::simulation
