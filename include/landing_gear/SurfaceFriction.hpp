#pragma once

#include <Eigen/Dense>

namespace liteaero::simulation {

struct FrictionCoefficients {
    float longitudinal_peak_nd    = 0.8f;
    float longitudinal_sliding_nd = 0.7f;
    float lateral_peak_nd         = 0.8f;
    float lateral_sliding_nd      = 0.7f;
};

class SurfaceFriction {
public:
    // Returns friction coefficients at the given NED position (m from local origin).
    [[nodiscard]] virtual FrictionCoefficients frictionCoefficients(
        const Eigen::Vector3f& position_ned_m) const = 0;

    virtual ~SurfaceFriction() = default;
};

}  // namespace liteaero::simulation
