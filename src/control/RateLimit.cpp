#include "control/RateLimit.hpp"

using namespace liteaerosim::control;

void RateLimit::setLower(float lim)
{
    lower_limit_ = lim;
    upper_limit_ = (upper_limit_ >= lim) ? upper_limit_ : lim;
}

void RateLimit::setUpper(float lim)
{
    upper_limit_ = lim;
    lower_limit_ = (lower_limit_ <= lim) ? lower_limit_ : lim;
}

void RateLimit::resetTo(float u)
{
    in_  = u;
    out_ = u;
    limited_lower_ = false;
    limited_upper_ = false;
}

float RateLimit::onStep(float u)
{
    limited_lower_ = false;
    limited_upper_ = false;

    float delta = u - in_;
    float delta_lim = delta;

    if (isLowerEnabled() && delta < dt_s_ * lower_limit_) {
        delta_lim = dt_s_ * lower_limit_;
        limited_lower_ = true;
    }

    if (isUpperEnabled() && delta > dt_s_ * upper_limit_) {
        delta_lim = dt_s_ * upper_limit_;
        limited_upper_ = true;
    }

    return out_ + delta_lim;
}

void RateLimit::onReset()
{
    limited_lower_ = false;
    limited_upper_ = false;
}

void RateLimit::onInitialize(const nlohmann::json& config)
{
    if (config.contains("dt_s"))          setDt(config.at("dt_s").get<float>());
    if (config.contains("lower_limit"))   setLower(config.at("lower_limit").get<float>());
    if (config.contains("upper_limit"))   setUpper(config.at("upper_limit").get<float>());
    if (config.value("lower_enabled", false)) enableLower();
    if (config.value("upper_enabled", false)) enableUpper();
}

nlohmann::json RateLimit::onSerializeJson() const
{
    return {
        {"in",            in_},
        {"out",           out_},
        {"dt_s",          dt_s_},
        {"lower_limit",   lower_limit_},
        {"upper_limit",   upper_limit_},
        {"lower_enabled", lower_enabled_},
        {"upper_enabled", upper_enabled_}
    };
}

void RateLimit::onDeserializeJson(const nlohmann::json& state)
{
    in_  = state.at("in").get<float>();
    out_ = state.at("out").get<float>();
    dt_s_          = state.at("dt_s").get<float>();
    lower_limit_   = state.at("lower_limit").get<float>();
    upper_limit_   = state.at("upper_limit").get<float>();
    lower_enabled_ = state.at("lower_enabled").get<bool>();
    upper_enabled_ = state.at("upper_enabled").get<bool>();
    limited_lower_ = false;
    limited_upper_ = false;
}
