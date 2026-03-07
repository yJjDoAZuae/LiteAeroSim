#define _USE_MATH_DEFINES
#include "aerodynamics/AeroPerformance.hpp"
#include "liteaerosim.pb.h"
#include <cmath>
#include <stdexcept>

namespace liteaerosim::aerodynamics {

AeroPerformance::AeroPerformance(float S_ref_m2, float ar, float e, float cd0, float cl_y_beta)
    : _S(S_ref_m2), _ar(ar), _e(e), _cd0(cd0),
      _k(1.f / (static_cast<float>(M_PI) * e * ar)),
      _cl_y_beta(cl_y_beta)
{}

AeroForces AeroPerformance::compute(float /*alpha_rad*/, float beta_rad,
                                    float q_inf_pa, float cl) const
{
    const float cdi = _k * cl * cl;
    const float cd  = _cd0 + cdi;
    const float cy  = _cl_y_beta * beta_rad;
    const float qS  = q_inf_pa * _S;
    return {-qS * cd, qS * cy, -qS * cl};
}

// ── Serialization ──────────────────────────────────────────────────────────────

nlohmann::json AeroPerformance::serializeJson() const {
    return {
        {"schema_version", 1},
        {"type",           "AeroPerformance"},
        {"s_ref_m2",       _S},
        {"ar",             _ar},
        {"e",              _e},
        {"cd0",            _cd0},
        {"cl_y_beta",      _cl_y_beta},
    };
}

AeroPerformance AeroPerformance::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1) {
        throw std::runtime_error("AeroPerformance::deserializeJson: unsupported schema_version");
    }
    return AeroPerformance(
        j.at("s_ref_m2").get<float>(),
        j.at("ar").get<float>(),
        j.at("e").get<float>(),
        j.at("cd0").get<float>(),
        j.at("cl_y_beta").get<float>()
    );
}

std::vector<uint8_t> AeroPerformance::serializeProto() const {
    las_proto::AeroPerformanceParams proto;
    proto.set_schema_version(1);
    proto.set_s_ref_m2(_S);
    proto.set_ar(_ar);
    proto.set_e(_e);
    proto.set_cd0(_cd0);
    proto.set_cl_y_beta(_cl_y_beta);
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

AeroPerformance AeroPerformance::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::AeroPerformanceParams proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
        throw std::runtime_error("AeroPerformance::deserializeProto: failed to parse bytes");
    }
    if (proto.schema_version() != 1) {
        throw std::runtime_error("AeroPerformance::deserializeProto: unsupported schema_version");
    }
    return AeroPerformance(
        proto.s_ref_m2(),
        proto.ar(),
        proto.e(),
        proto.cd0(),
        proto.cl_y_beta()
    );
}

} // namespace liteaerosim::aerodynamics
