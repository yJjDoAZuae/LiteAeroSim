#pragma once

#include <landing_gear/SurfaceFriction.hpp>
#include <nlohmann/json.hpp>

namespace liteaero::simulation {

// Uniform surface friction model — returns the same FrictionCoefficients
// regardless of position.  Represents a homogeneous runway surface.
class SurfaceFrictionUniform : public SurfaceFriction {
public:
    // Named surface-type constructors.
    static SurfaceFrictionUniform pavement(bool wet = false);
    static SurfaceFrictionUniform grass(bool wet = false);
    static SurfaceFrictionUniform dirt(bool wet = false);
    static SurfaceFrictionUniform gravel(bool wet = false);

    // Custom coefficient constructor.
    explicit SurfaceFrictionUniform(FrictionCoefficients coeff = {});

    [[nodiscard]] FrictionCoefficients frictionCoefficients(
        const Eigen::Vector3f& position_ned_m) const override;

    [[nodiscard]] nlohmann::json serializeJson()                           const;
    static SurfaceFrictionUniform deserializeJson(const nlohmann::json&   j);

private:
    FrictionCoefficients _coeff;
};

}  // namespace liteaero::simulation
