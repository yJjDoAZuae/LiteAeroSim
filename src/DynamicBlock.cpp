#include "DynamicBlock.hpp"

#include <stdexcept>
#include <string>

namespace liteaerosim {

void DynamicBlock::initialize(const nlohmann::json& config) {
    onInitialize(config);
}

void DynamicBlock::reset() {
    in_  = 0.0f;
    out_ = 0.0f;
    onReset();
}

float DynamicBlock::step(float u) {
    in_  = u;
    out_ = onStep(u);
    if (logger_) {
        onLog(*logger_);
    }
    return out_;
}

nlohmann::json DynamicBlock::serialize() const {
    nlohmann::json state = onSerialize();
    state["schema_version"] = schemaVersion();
    state["type"]           = typeName();
    state["in"]             = in_;
    state["out"]            = out_;
    return state;
}

void DynamicBlock::deserialize(const nlohmann::json& state) {
    validateSchema(state);
    in_  = state.at("in").get<float>();
    out_ = state.at("out").get<float>();
    onDeserialize(state);
}

void DynamicBlock::attachLogger(ILogger* logger) noexcept {
    logger_ = logger;
}

void DynamicBlock::validateSchema(const nlohmann::json& state) const {
    const int stored  = state.at("schema_version").get<int>();
    const int current = schemaVersion();
    if (stored != current) {
        throw std::runtime_error(
            std::string("DynamicBlock::deserialize: schema version mismatch for ") +
            typeName() + ": stored=" + std::to_string(stored) +
            " current=" + std::to_string(current));
    }
}

} // namespace liteaerosim
