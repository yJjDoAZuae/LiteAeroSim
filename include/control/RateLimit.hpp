#pragma once

#include "SisoElement.hpp"

namespace liteaerosim::control {

class RateLimit : public liteaerosim::SisoElement {
public:
    RateLimit() :
        lower_limit_(0.0f),
        upper_limit_(0.0f),
        limited_lower_(false),
        limited_upper_(false),
        lower_enabled_(false),
        upper_enabled_(false),
        dt_s_(1.0f)
    {}

    void disableLower() { lower_enabled_ = false; }
    void disableUpper() { upper_enabled_ = false; }
    void enableLower()  { lower_enabled_ = true; }
    void enableUpper()  { upper_enabled_ = true; }
    void disable()      { disableLower(); disableUpper(); }
    void enable()       { enableLower();  enableUpper(); }

    void setLower(float lim);
    void setUpper(float lim);
    void set(float lower_lim, float upper_lim) { setLower(lower_lim); setUpper(upper_lim); }
    void setDt(float dt_s)  { dt_s_ = (dt_s > 1e-6f) ? dt_s : 1.0f; }

    float lowerLimit()     const { return lower_limit_; }
    float upperLimit()     const { return upper_limit_; }
    bool  isLimitedLower() const { return limited_lower_; }
    bool  isLimitedUpper() const { return limited_upper_; }
    bool  isLimited()      const { return limited_lower_ || limited_upper_; }
    bool  isLowerEnabled() const { return lower_enabled_; }
    bool  isUpperEnabled() const { return upper_enabled_; }
    float dt_s()           const { return dt_s_; }

    /// Warm-start: set output (and input) to u without stepping.
    void resetTo(float u);

protected:
    float onStep(float u) override;
    void  onReset() override;
    void  onInitialize(const nlohmann::json& config) override;
    nlohmann::json onSerializeJson() const override;
    void  onDeserializeJson(const nlohmann::json& state) override;
    int   schemaVersion() const override { return 1; }
    const char* typeName() const override { return "RateLimit"; }

private:
    float lower_limit_;
    float upper_limit_;
    bool  limited_lower_;
    bool  limited_upper_;
    bool  lower_enabled_;
    bool  upper_enabled_;
    float dt_s_;
};

} // namespace liteaerosim::control
