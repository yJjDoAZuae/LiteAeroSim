#pragma once

#include "control/control.hpp"
#include "SisoElement.hpp"
#include "control/Limit.hpp"
#include "control/Antiwindup.hpp"
#include <vector>

namespace liteaerosim::control {

class Derivative : public liteaerosim::SisoElement {
public:
    Derivative() :
        dt_s_(1.0f),
        tau_s_(0.0f),
        method_(DiscretizationMethod::ForwardEuler)
    {}

    /// Warm-start: set input to u, output to uDot.
    void resetTo(float u, float uDot = 0.0f);

    void setDt(float dt)                        { dt_s_  = (dt > 1e-6f) ? dt : 1.0f; }
    void setTau(float tau)                      { tau_s_ = tau; }
    void setMethod(DiscretizationMethod method) { method_ = method; }
    float dt_s()  const { return dt_s_; }
    float tau_s() const { return tau_s_; }

protected:
    float onStep(float u) override;
    void  onReset() override {}
    void  onInitialize(const nlohmann::json& config) override;
    nlohmann::json onSerializeJson() const override;
    void  onDeserializeJson(const nlohmann::json& state) override;
    int   schemaVersion() const override { return 1; }
    const char* typeName() const override { return "Derivative"; }

private:
    Limit limit_;
    float dt_s_;
    float tau_s_;
    DiscretizationMethod method_;
};

} // namespace liteaerosim::control
