
#pragma once

#include "control/Filter.hpp"
#include "control/Limit.hpp"
#include <nlohmann/json.hpp>

namespace liteaerosim::control {

class FilterSS2Clip : public Filter {

public:
    FilterSS2Clip();
    ~FilterSS2Clip() override = default;

    Limit valLimit;
    Limit rateLimit;

    void copy(const FilterSS2Clip& filt);

    // IIR filter design
    void setLowPassFirstIIR (float dt_s, float tau);
    void setLowPassSecondIIR(float dt_s, float wn_rad_s, float zeta, float tau_zero);
    void setHighPassFirstIIR(float dt_s, float tau);
    void setHighPassSecondIIR(float dt_s, float wn_rad_s, float zeta, float c_zero);
    void setDerivIIR        (float dt_s, float tau);
    void setNotchSecondIIR  (float dt_s, float wn_rad_s, float zeta_den, float zeta_num);

    float dcGain() const override;

    // State-space matrix accessors (read-only)
    Mat22 phi()   const { return phi_; }
    Mat21 gamma() const { return gamma_; }
    Mat12 h()     const { return h_; }
    Mat11 j()     const { return j_; }
    Mat21 x()     const { return x_; }

    void resetToInput (float in_val)  override;
    void resetToOutput(float out_val) override;

    // Directly restore the filter internal state vector.
    // Exact warm-start: restores x without the steady-state backsolve assumption.
    void resetState(const Mat21& x);

    uint8_t  order()     const override { return order_; }
    float    dt_s()      const          { return dt_s_; }
    uint16_t errorCode() const override { return error_code_; }

    Mat22 controlGrammian() const;
    Mat22 observeGrammian() const;

protected:
    float          onStep(float u)                                override;
    void           onInitialize(const nlohmann::json& config)     override;
    nlohmann::json onSerializeJson()                        const override;
    void           onDeserializeJson(const nlohmann::json& state) override;
    int            schemaVersion()  const override { return kSchemaVersion_; }
    const char*    typeName()       const override { return "FilterSS2Clip"; }

private:
    static constexpr int kSchemaVersion_ = 1;

    void backsolve1(float in_prev, float out_prev, float in_curr, float out_curr);
    void backsolve2(float in_prev, float out_prev, float in_curr, float out_curr);
    void backsolve (float in_prev, float out_prev, float in_curr, float out_curr);

    // 2nd order state-space realization matrices
    Mat22 phi_;
    Mat21 gamma_;
    Mat12 h_;
    Mat11 j_;

    // 2nd order state vector
    Mat21 x_;

    uint8_t  order_      = 0;
    float    dt_s_       = 1.0f;
    uint16_t error_code_ = 0;
};

}  // namespace liteaerosim::control
