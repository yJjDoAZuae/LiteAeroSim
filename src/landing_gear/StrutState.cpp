#include "landing_gear/StrutState.hpp"
#include "liteaerosim.pb.h"
#include <stdexcept>

namespace liteaero::simulation {

nlohmann::json StrutState::serializeJson() const {
    return {
        {"schema_version",            1},
        {"strut_deflection_m",        strut_deflection_m},
        {"strut_deflection_rate_mps", strut_deflection_rate_mps},
        {"wheel_speed_rps",           wheel_speed_rps},
    };
}

void StrutState::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("StrutState::deserializeJson: unsupported schema_version");
    strut_deflection_m        = j.at("strut_deflection_m").get<float>();
    strut_deflection_rate_mps = j.at("strut_deflection_rate_mps").get<float>();
    wheel_speed_rps           = j.at("wheel_speed_rps").get<float>();
}

std::vector<uint8_t> StrutState::serializeProto() const {
    las_proto::WheelUnitState proto;
    proto.set_strut_deflection_m(strut_deflection_m);
    proto.set_strut_deflection_rate_mps(strut_deflection_rate_mps);
    proto.set_wheel_speed_rps(wheel_speed_rps);
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void StrutState::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::WheelUnitState proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())))
        throw std::runtime_error("StrutState::deserializeProto: failed to parse");
    strut_deflection_m        = proto.strut_deflection_m();
    strut_deflection_rate_mps = proto.strut_deflection_rate_mps();
    wheel_speed_rps           = proto.wheel_speed_rps();
}

}  // namespace liteaero::simulation
