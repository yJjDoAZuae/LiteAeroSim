#include "environment/Wind.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace liteaerosim::environment {

Wind::Wind(const WindConfig& config)
    : config_(config)
{}

float Wind::magnitude_at(float altitude_m) const {
    switch (config_.profile) {
        case WindConfig::Profile::Constant:
            return config_.speed_mps;

        case WindConfig::Profile::PowerLaw: {
            float h = std::max(altitude_m, 0.1f);  // avoid pow(0, alpha)
            return config_.speed_mps
                   * std::pow(h / config_.reference_altitude_m,
                              config_.hellmann_alpha_nd);
        }

        case WindConfig::Profile::Logarithmic: {
            float z0 = config_.roughness_length_m;
            float h  = std::max(altitude_m, z0 * 1.001f);  // avoid log singularity
            return config_.speed_mps
                   * std::log(h / z0)
                   / std::log(config_.reference_altitude_m / z0);
        }
    }
    return 0.f;  // unreachable
}

Eigen::Vector3f Wind::wind_NED_mps(float altitude_m,
                                   const Eigen::Vector3f& /*position_NED_m*/) const {
    return config_.direction_NED_unit * magnitude_at(altitude_m);
}

const WindConfig& Wind::config() const {
    return config_;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------
nlohmann::json Wind::serializeJson() const {
    return {
        {"schema_version",        config_.schema_version},
        {"direction_ned_x",       config_.direction_NED_unit.x()},
        {"direction_ned_y",       config_.direction_NED_unit.y()},
        {"direction_ned_z",       config_.direction_NED_unit.z()},
        {"speed_mps",             config_.speed_mps},
        {"reference_altitude_m",  config_.reference_altitude_m},
        {"profile",               static_cast<int>(config_.profile)},
        {"hellmann_alpha_nd",     config_.hellmann_alpha_nd},
        {"roughness_length_m",    config_.roughness_length_m},
    };
}

void Wind::deserializeJson(const nlohmann::json& j) {
    int sv = j.at("schema_version").get<int>();
    if (sv != 1) {
        throw std::runtime_error("Wind: unsupported schema_version " + std::to_string(sv));
    }
    WindConfig cfg;
    cfg.schema_version       = sv;
    cfg.direction_NED_unit   = {j.at("direction_ned_x").get<float>(),
                                j.at("direction_ned_y").get<float>(),
                                j.at("direction_ned_z").get<float>()};
    cfg.speed_mps            = j.at("speed_mps").get<float>();
    cfg.reference_altitude_m = j.at("reference_altitude_m").get<float>();
    cfg.profile              = static_cast<WindConfig::Profile>(j.at("profile").get<int>());
    cfg.hellmann_alpha_nd    = j.at("hellmann_alpha_nd").get<float>();
    cfg.roughness_length_m   = j.at("roughness_length_m").get<float>();
    config_ = cfg;
}

} // namespace liteaerosim::environment
