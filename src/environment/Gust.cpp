#include "environment/Gust.hpp"
#include <cmath>

namespace liteaero::simulation {

static constexpr double kPi = 3.14159265358979323846;

void Gust::trigger(const GustConfig& config, double trigger_time_s) {
    config_          = config;
    trigger_time_s_  = trigger_time_s;
    armed_           = true;
    active_          = false;
    end_time_s_      = 0.0;
}

Eigen::Vector3f Gust::step(double time_s, float airspeed_mps) {
    if (!armed_ && !active_) {
        return Eigen::Vector3f::Zero();
    }

    // Arm fires on first step at or after trigger time
    if (armed_ && time_s >= trigger_time_s_) {
        float Va     = std::max(airspeed_mps, 0.1f);
        end_time_s_  = trigger_time_s_
                       + 2.0 * static_cast<double>(config_.gradient_dist_m) / Va;
        armed_  = false;
        active_ = true;
    }

    if (!active_) {
        return Eigen::Vector3f::Zero();
    }

    if (time_s >= end_time_s_) {
        active_ = false;
        return Eigen::Vector3f::Zero();
    }

    float Va  = std::max(airspeed_mps, 0.1f);
    float H   = config_.gradient_dist_m;
    float Vg  = config_.amplitude_mps;
    double dt = time_s - trigger_time_s_;

    float v = (Vg / 2.f) * (1.f - static_cast<float>(
                  std::cos(kPi * Va * dt / H)));

    return config_.direction_body * v;
}

bool Gust::is_active() const {
    return active_;
}

void Gust::reset() {
    armed_  = false;
    active_ = false;
}

} // namespace liteaero::simulation
