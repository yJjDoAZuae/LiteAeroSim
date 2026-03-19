#include "control/Derivative.hpp"

using namespace liteaerosim::control;

float Derivative::onStep(float u)
{
    tau_s_ = (tau_s_ > 4.0f * dt_s_) ? tau_s_ : 4.0f * dt_s_;

    float next;
    switch (method_) {
    case DiscretizationMethod::BackwardEuler:
        next = limit_.step(tau_s_ / (tau_s_ + dt_s_) * out_ + (u - in_) / (tau_s_ + dt_s_));
        break;
    case DiscretizationMethod::ForwardEuler:
        next = limit_.step(out_ * (tau_s_ - dt_s_) / tau_s_ + (u - in_) / tau_s_);
        break;
    case DiscretizationMethod::Bilinear:
        next = limit_.step(out_ * (2.0f * tau_s_ - dt_s_) / (2.0f * tau_s_ + dt_s_)
                           + 0.5f * (u + in_) * 2.0f / (2.0f * tau_s_ + dt_s_));
        break;
    default:
        next = out_;
        break;
    }

    return next;
}

void Derivative::resetTo(float u, float uDot)
{
    out_ = limit_.step(uDot);
    in_  = u;
}

void Derivative::onInitialize(const nlohmann::json& config)
{
    dt_s_   = config.at("dt_s").get<float>();
    tau_s_  = config.at("tau_s").get<float>();
    method_ = static_cast<DiscretizationMethod>(config.at("method").get<int>());
}

nlohmann::json Derivative::onSerializeJson() const
{
    return {
        {"in",     in_},
        {"out",    out_},
        {"dt_s",   dt_s_},
        {"tau_s",  tau_s_},
        {"method", static_cast<int>(method_)}
    };
}

void Derivative::onDeserializeJson(const nlohmann::json& state)
{
    in_     = state.at("in").get<float>();
    out_    = state.at("out").get<float>();
    dt_s_   = state.at("dt_s").get<float>();
    tau_s_  = state.at("tau_s").get<float>();
    method_ = static_cast<DiscretizationMethod>(state.at("method").get<int>());
}
