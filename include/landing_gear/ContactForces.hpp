#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <vector>

namespace liteaero::simulation {

struct ContactForces {
    Eigen::Vector3f force_body_n   = Eigen::Vector3f::Zero();  // body-frame force (N)
    Eigen::Vector3f moment_body_nm = Eigen::Vector3f::Zero();  // body-frame moment (N·m)
    bool            weight_on_wheels = false;

    [[nodiscard]] nlohmann::json       serializeJson()                               const;
    void                               deserializeJson(const nlohmann::json&          j);
    [[nodiscard]] std::vector<uint8_t> serializeProto()                              const;
    void                               deserializeProto(const std::vector<uint8_t>& bytes);
};

}  // namespace liteaero::simulation
