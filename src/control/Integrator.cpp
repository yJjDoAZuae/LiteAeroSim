#include "control/Integrator.hpp"

using namespace liteaerosim::control;

float Integrator::onStep(float u)
{
    bool awActive = false;
    for (size_t k = 0; k < aw.size(); k++) {
        awActive |= aw.at(k).isActive();
    }

    float next;
    if (!awActive) {
        switch (method_) {
        case DiscretizationMethod::BackwardEuler:
            next = limit_.step(out_ + u * dt_s_);
            break;
        case DiscretizationMethod::ForwardEuler:
            next = limit_.step(out_ + in_ * dt_s_);
            break;
        case DiscretizationMethod::Bilinear:
            next = limit_.step(out_ + 0.5f * (u + in_) * dt_s_);
            break;
        default:
            next = out_;
            break;
        }
    } else {
        next = limit_.step(out_);
    }

    return next;
}

void Integrator::resetTo(float u)
{
    out_ = limit_.step(u);
    in_  = u;
}

void Integrator::onReset()
{
    // in_ and out_ already zeroed by SisoElement::reset()
}

void Integrator::onInitialize(const nlohmann::json& config)
{
    dt_s_   = config.at("dt_s").get<float>();
    method_ = static_cast<DiscretizationMethod>(config.at("method").get<int>());
}

nlohmann::json Integrator::onSerializeJson() const
{
    nlohmann::json aw_array = nlohmann::json::array();
    for (const auto& detector : aw) {
        aw_array.push_back(detector.serializeJson());
    }
    return {
        {"in",         in_},
        {"out",        out_},
        {"dt_s",       dt_s_},
        {"method",     static_cast<int>(method_)},
        {"antiwindup", aw_array}
    };
}

void Integrator::onDeserializeJson(const nlohmann::json& state)
{
    in_     = state.at("in").get<float>();
    out_    = state.at("out").get<float>();
    dt_s_   = state.at("dt_s").get<float>();
    method_ = static_cast<DiscretizationMethod>(state.at("method").get<int>());

    if (state.contains("antiwindup")) {
        const auto& aw_array = state.at("antiwindup");
        aw.resize(aw_array.size());
        for (size_t k = 0; k < aw_array.size(); ++k) {
            aw[k].deserializeJson(aw_array[k]);
        }
    }
}
