
#pragma once

#include "control/Filter.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace liteaerosim::control {

class FilterTF : public Filter {

public:
    FilterTF();
    FilterTF(const FilterTF& filt);
    ~FilterTF() override = default;

    void copy(const FilterTF& filt);

    void setButterworthIIR(char order, float dt, float wn_rps);

    uint8_t order() const override { return static_cast<uint8_t>(den_.rows() - 1); }

    void resetToInput (float in_val)  override;
    void resetToOutput(float out_val) override;
    float    dcGain()    const override;
    uint16_t errorCode() const override { return error_code_; }

    FiltVectorXf num() const { return num_; }
    FiltVectorXf den() const { return den_; }

protected:
    float          onStep(float u)                                override;
    void           onInitialize(const nlohmann::json& config)     override;
    nlohmann::json onSerializeJson()                        const override;
    void           onDeserializeJson(const nlohmann::json& state) override;
    int            schemaVersion()  const override { return kSchemaVersion_; }
    const char*    typeName()       const override { return "FilterTF"; }

private:
    static constexpr int kSchemaVersion_ = 1;

    FiltVectorXf num_;
    FiltVectorXf den_;
    FiltVectorXf u_buff_;
    FiltVectorXf y_buff_;
    uint16_t error_code_ = 0;
};

}  // namespace liteaerosim::control
