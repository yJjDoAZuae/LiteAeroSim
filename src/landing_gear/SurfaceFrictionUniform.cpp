#include "landing_gear/SurfaceFrictionUniform.hpp"
#include <stdexcept>

namespace liteaero::simulation {

SurfaceFrictionUniform SurfaceFrictionUniform::pavement(bool wet) {
    if (wet) return SurfaceFrictionUniform{{0.40f, 0.35f, 0.40f, 0.35f}};
    return SurfaceFrictionUniform{{0.80f, 0.70f, 0.80f, 0.70f}};
}

SurfaceFrictionUniform SurfaceFrictionUniform::grass(bool wet) {
    if (wet) return SurfaceFrictionUniform{{0.30f, 0.25f, 0.30f, 0.25f}};
    return SurfaceFrictionUniform{{0.40f, 0.35f, 0.40f, 0.35f}};
}

SurfaceFrictionUniform SurfaceFrictionUniform::dirt(bool wet) {
    if (wet) return SurfaceFrictionUniform{{0.25f, 0.20f, 0.25f, 0.20f}};
    return SurfaceFrictionUniform{{0.50f, 0.40f, 0.50f, 0.40f}};
}

SurfaceFrictionUniform SurfaceFrictionUniform::gravel(bool wet) {
    if (wet) return SurfaceFrictionUniform{{0.35f, 0.30f, 0.35f, 0.30f}};
    return SurfaceFrictionUniform{{0.60f, 0.50f, 0.60f, 0.50f}};
}

SurfaceFrictionUniform::SurfaceFrictionUniform(FrictionCoefficients coeff)
    : _coeff(coeff) {}

FrictionCoefficients SurfaceFrictionUniform::frictionCoefficients(
    const Eigen::Vector3f& /*position_ned_m*/) const {
    return _coeff;
}

nlohmann::json SurfaceFrictionUniform::serializeJson() const {
    return {
        {"schema_version",          1},
        {"longitudinal_peak_nd",    _coeff.longitudinal_peak_nd},
        {"longitudinal_sliding_nd", _coeff.longitudinal_sliding_nd},
        {"lateral_peak_nd",         _coeff.lateral_peak_nd},
        {"lateral_sliding_nd",      _coeff.lateral_sliding_nd},
    };
}

SurfaceFrictionUniform SurfaceFrictionUniform::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error(
            "SurfaceFrictionUniform::deserializeJson: unsupported schema_version");
    FrictionCoefficients c;
    c.longitudinal_peak_nd    = j.at("longitudinal_peak_nd").get<float>();
    c.longitudinal_sliding_nd = j.at("longitudinal_sliding_nd").get<float>();
    c.lateral_peak_nd         = j.at("lateral_peak_nd").get<float>();
    c.lateral_sliding_nd      = j.at("lateral_sliding_nd").get<float>();
    return SurfaceFrictionUniform{c};
}

}  // namespace liteaero::simulation
