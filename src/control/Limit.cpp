#include "control/Limit.hpp"

using namespace liteaerosim::control;

float Limit::onStep(float u)
{
    float out = u;
    limited_lower_ = false;
    limited_upper_ = false;

    if (isLowerEnabled() && out < lowerLimit()) {
        out = lowerLimit();
        limited_lower_ = true;
    }

    if (isUpperEnabled() && out > upperLimit()) {
        out = upperLimit();
        limited_upper_ = true;
    }

    return out;
}

void Limit::onReset()
{
    limited_lower_ = false;
    limited_upper_ = false;
}

void Limit::setLower(float lim)
{
    lower_limit_ = lim;
    upper_limit_ = (upper_limit_ >= lim) ? upper_limit_ : lim;
    step(in_);
}

void Limit::setUpper(float lim)
{
    upper_limit_ = lim;
    lower_limit_ = (lower_limit_ <= lim) ? lower_limit_ : lim;
    step(in_);
}

void Limit::onInitialize(const nlohmann::json& config)
{
    if (config.contains("lower_limit")) setLower(config.at("lower_limit").get<float>());
    if (config.contains("upper_limit")) setUpper(config.at("upper_limit").get<float>());
    if (config.value("lower_enabled", false)) enableLower();
    if (config.value("upper_enabled", false)) enableUpper();
}

nlohmann::json Limit::onSerializeJson() const
{
    return {
        {"in",             in_},
        {"out",            out_},
        {"lower_limit",    lower_limit_},
        {"upper_limit",    upper_limit_},
        {"lower_enabled",  lower_enabled_},
        {"upper_enabled",  upper_enabled_}
    };
}

void Limit::onDeserializeJson(const nlohmann::json& state)
{
    in_  = state.at("in").get<float>();
    out_ = state.at("out").get<float>();
    lower_limit_  = state.at("lower_limit").get<float>();
    upper_limit_  = state.at("upper_limit").get<float>();
    lower_enabled_ = state.at("lower_enabled").get<bool>();
    upper_enabled_ = state.at("upper_enabled").get<bool>();
    // Recompute limited flags from restored state
    limited_lower_ = lower_enabled_ && (in_ < lower_limit_);
    limited_upper_ = upper_enabled_ && (in_ > upper_limit_);
}
