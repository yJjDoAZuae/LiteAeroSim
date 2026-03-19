#pragma once

#include "SisoElement.hpp"

namespace liteaerosim::control {

class Unwrap : public liteaerosim::SisoElement {
public:
    /// Warm-start: set input, output, and reference to u.
    void resetTo(float u) { in_ = u; out_ = u; ref_ = u; }

    /// Store a reference value used by the next single-arg step() call.
    void setReference(float ref) { ref_ = ref; }

    using SisoElement::step;

    /// Unwrap u relative to an externally provided reference value.
    /// Sets the stored reference to ref, then calls the NVI step.
    float step(float u, float ref);

protected:
    float onStep(float u) override;
    void  onReset() override { ref_ = 0.0f; }
    void  onInitialize(const nlohmann::json&) override {}
    nlohmann::json onSerializeJson() const override;
    void  onDeserializeJson(const nlohmann::json& state) override;
    int   schemaVersion() const override { return 1; }
    const char* typeName() const override { return "Unwrap"; }

private:
    float ref_ = 0.0f;
};

} // namespace liteaerosim::control
