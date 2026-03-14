#pragma once
#include <Eigen/Dense>

namespace liteaerosim::environment {

struct GustConfig {
    float           amplitude_mps   = 0.f;                    // peak gust velocity (m/s)
    float           gradient_dist_m = 50.f;                   // half-gust gradient distance H (m)
    Eigen::Vector3f direction_body  = {0.f, 0.f, 1.f};       // unit vector in body frame; default: vertical
    int             schema_version  = 1;
};

class Gust {
public:
    Gust() = default;

    // Arms the gust. It fires on the first step() call at or after trigger_time_s.
    void trigger(const GustConfig& config, double trigger_time_s);

    // Returns the gust velocity vector in body frame (m/s) for the given simulation time
    // and aircraft true airspeed. Returns {0,0,0} when no gust is active.
    Eigen::Vector3f step(double time_s, float airspeed_mps);

    // True while the gust is active (between trigger and end of gust profile).
    bool is_active() const;

    // Disarms any pending or in-progress gust.
    void reset();

private:
    GustConfig config_;
    double trigger_time_s_ = 0.0;
    double end_time_s_     = 0.0;
    bool   armed_          = false;
    bool   active_         = false;
};

} // namespace liteaerosim::environment
