
#pragma once

#include "control/Filter.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace liteaerosim::control {

class FilterTF2 : public Filter {

public:
    FilterTF2();
    FilterTF2(const FilterTF2& filt);
    ~FilterTF2() override = default;

    void copy(const FilterTF2& filt);

    // IIR filter design
    void setLowPassFirstIIR (float dt, float tau);
    void setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero);
    void setHighPassFirstIIR(float dt, float tau);
    void setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero);
    void setDerivIIR        (float dt, float tau);
    void setNotchSecondIIR  (float dt, float wn_rps, float zeta_den, float zeta_num);

    void resetToInput (float in_val)  override;
    void resetToOutput(float out_val) override;

    float    dcGain()    const override;
    uint8_t  order()     const override { return order_; }
    uint16_t errorCode() const override { return error_code_; }

    Vec3 num() const { return num_; }
    Vec3 den() const { return den_; }

protected:
    float          onStep(float u)                                override;
    void           onInitialize(const nlohmann::json& config)     override;
    nlohmann::json onSerializeJson()                        const override;
    void           onDeserializeJson(const nlohmann::json& state) override;
    int            schemaVersion()  const override { return kSchemaVersion_; }
    const char*    typeName()       const override { return "FilterTF2"; }

private:
    static constexpr int kSchemaVersion_ = 1;

    Vec3 num_;
    Vec3 den_;

    Vec3 u_buff_;
    Vec3 y_buff_;

    uint8_t  order_      = 0;
    uint16_t error_code_ = 0;
};

}  // namespace liteaerosim::control
