
#pragma once

#include "control/Filter.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace liteaerosim::control {

class FilterFIR : public Filter {

public:
    FilterFIR();
    FilterFIR(const FilterFIR& filt);
    ~FilterFIR() override = default;

    void copy(const FilterFIR& filt);

    uint8_t order() const override { return static_cast<uint8_t>(num_.rows() - 1); }

    void setAverageFIR(char order);
    void setExpFIR(char order, float dt, float tau);

    void resetToInput (float in_val)  override;
    void resetToOutput(float out_val) override;
    float    dcGain()    const override;
    uint16_t errorCode() const override { return error_code_; }

protected:
    float          onStep(float u)                                override;
    void           onInitialize(const nlohmann::json& config)     override;
    nlohmann::json onSerializeJson()                        const override;
    void           onDeserializeJson(const nlohmann::json& state) override;
    int            schemaVersion()  const override { return kSchemaVersion_; }
    const char*    typeName()       const override { return "FilterFIR"; }

private:
    static constexpr int kSchemaVersion_ = 1;

    FiltVectorXf num_;
    FiltVectorXf u_buff_;
    uint16_t error_code_ = 0;
};

}  // namespace liteaerosim::control
