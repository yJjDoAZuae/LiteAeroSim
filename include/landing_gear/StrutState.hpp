#pragma once

#include <nlohmann/json.hpp>
#include <cstdint>
#include <vector>

namespace liteaero::simulation {

struct StrutState {
    float strut_deflection_m        = 0.0f;  // m
    float strut_deflection_rate_mps = 0.0f;  // m/s
    float wheel_speed_rps           = 0.0f;  // rad/s

    [[nodiscard]] nlohmann::json       serializeJson()                               const;
    void                               deserializeJson(const nlohmann::json&          j);
    [[nodiscard]] std::vector<uint8_t> serializeProto()                              const;
    void                               deserializeProto(const std::vector<uint8_t>& bytes);
};

}  // namespace liteaero::simulation
