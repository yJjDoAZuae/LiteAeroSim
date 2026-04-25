#include "physics/ContactForces.hpp"
#include "liteaerosim.pb.h"
#include <stdexcept>

namespace liteaero::simulation {

nlohmann::json ContactForces::serializeJson() const {
    return {
        {"schema_version", 1},
        {"force_body_n",   {{"x", force_body_n.x()}, {"y", force_body_n.y()}, {"z", force_body_n.z()}}},
        {"moment_body_nm", {{"x", moment_body_nm.x()}, {"y", moment_body_nm.y()}, {"z", moment_body_nm.z()}}},
        {"weight_on_wheels", weight_on_wheels},
    };
}

void ContactForces::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("ContactForces::deserializeJson: unsupported schema_version");
    const auto& f  = j.at("force_body_n");
    force_body_n   = {f.at("x").get<float>(), f.at("y").get<float>(), f.at("z").get<float>()};
    const auto& m  = j.at("moment_body_nm");
    moment_body_nm = {m.at("x").get<float>(), m.at("y").get<float>(), m.at("z").get<float>()};
    weight_on_wheels = j.at("weight_on_wheels").get<bool>();
}

std::vector<uint8_t> ContactForces::serializeProto() const {
    las_proto::ContactForcesState proto;
    proto.set_schema_version(1);
    auto* f = proto.mutable_force_body_n();
    f->set_x(force_body_n.x());
    f->set_y(force_body_n.y());
    f->set_z(force_body_n.z());
    auto* m = proto.mutable_moment_body_nm();
    m->set_x(moment_body_nm.x());
    m->set_y(moment_body_nm.y());
    m->set_z(moment_body_nm.z());
    proto.set_weight_on_wheels(weight_on_wheels);
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void ContactForces::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::ContactForcesState proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())))
        throw std::runtime_error("ContactForces::deserializeProto: failed to parse");
    if (proto.schema_version() != 1)
        throw std::runtime_error("ContactForces::deserializeProto: unsupported schema_version");
    force_body_n     = {proto.force_body_n().x(),   proto.force_body_n().y(),   proto.force_body_n().z()};
    moment_body_nm   = {proto.moment_body_nm().x(), proto.moment_body_nm().y(), proto.moment_body_nm().z()};
    weight_on_wheels = proto.weight_on_wheels();
}

}  // namespace liteaero::simulation
