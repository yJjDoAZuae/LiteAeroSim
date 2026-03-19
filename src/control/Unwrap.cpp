#include "control/Unwrap.hpp"
#include "math/math_util.hpp"

using namespace liteaerosim::control;

float Unwrap::onStep(float u) {
    float y = ref_ + MathUtil::wrapToPi(u - ref_);
    ref_ = y;
    return y;
}

float Unwrap::step(float u, float ref) {
    ref_ = ref;
    return SisoElement::step(u);
}

nlohmann::json Unwrap::onSerializeJson() const {
    return {
        {"in",  in_},
        {"out", out_},
        {"ref", ref_}
    };
}

void Unwrap::onDeserializeJson(const nlohmann::json& state) {
    in_  = state.at("in").get<float>();
    out_ = state.at("out").get<float>();
    ref_ = state.contains("ref") ? state.at("ref").get<float>() : out_;
}
