#pragma once

#include "control/control.hpp"
#include "SisoElement.hpp"
#include "control/Limit.hpp"
#include "control/Antiwindup.hpp"
#include <vector>

namespace liteaerosim::control {

class Integrator : public liteaerosim::SisoElement {
public:
    Integrator() :
        dt_s_(1.0f),
        method_(DiscretizationMethod::ForwardEuler)
    {}

    std::vector<Antiwindup> aw;

    /// Warm-start: set output (and input) to u.
    void resetTo(float u);

    void setDt(float dt)                        { dt_s_ = (dt > 1e-6f) ? dt : 1.0f; }
    void setMethod(DiscretizationMethod method) { method_ = method; }
    float dt_s() const { return dt_s_; }

protected:
    float onStep(float u) override;
    void  onReset() override;
    void  onInitialize(const nlohmann::json& config) override;
    nlohmann::json onSerializeJson() const override;
    void  onDeserializeJson(const nlohmann::json& state) override;
    int   schemaVersion() const override { return 1; }
    const char* typeName() const override { return "Integrator"; }

private:
    Limit limit_;
    float dt_s_;
    DiscretizationMethod method_;
};

} // namespace liteaerosim::control
