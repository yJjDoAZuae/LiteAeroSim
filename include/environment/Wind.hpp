#pragma once
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace liteaerosim::environment {

struct WindConfig {
    Eigen::Vector3f direction_NED_unit  = {1.f, 0.f, 0.f};  // unit vector in NED
    float           speed_mps           = 0.f;               // reference magnitude (m/s)
    float           reference_altitude_m = 10.f;             // altitude of speed_mps (m)
    enum class Profile { Constant, PowerLaw, Logarithmic } profile = Profile::Constant;
    float           hellmann_alpha_nd   = 0.14f;             // power-law exponent
    float           roughness_length_m  = 0.03f;             // log-law z₀ (m)
    int             schema_version      = 1;
};

class Wind {
public:
    Wind() = default;
    explicit Wind(const WindConfig& config);

    // Returns the ambient wind vector in NED frame at the given geometric altitude (m).
    // position_NED_m is provided for future spatially-varying extensions.
    Eigen::Vector3f wind_NED_mps(float altitude_m,
                                 const Eigen::Vector3f& position_NED_m = {}) const;

    const WindConfig& config() const;

    nlohmann::json serializeJson() const;
    void deserializeJson(const nlohmann::json& j);

private:
    WindConfig config_;
    float magnitude_at(float altitude_m) const;
};

} // namespace liteaerosim::environment
