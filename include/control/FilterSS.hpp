
#pragma once

#include "control/control.hpp"
#include "control/Filter.hpp"
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace liteaerosim::control {

class FilterSS : public Filter {

public:
    FilterSS();
    FilterSS(const FilterSS& filt);
    ~FilterSS() override;

    void copy(const FilterSS& filt);

    void setButterworthIIR(uint8_t order, float dt, float wn_rps);

    uint8_t order() const override;

    void resetToInput (float in_val)  override;
    void resetToOutput(float out_val) override;
    float    dcGain()    const override;
    uint16_t errorCode() const override { return error_code_; }

    MatNN phi()   const { return phi_; }
    MatN1 gamma() const { return gamma_; }
    Mat1N h()     const { return h_; }
    Mat11 j()     const { return j_; }
    MatN1 x()     const { return x_; }

    MatNN controlGrammian() const;
    MatNN observeGrammian() const;

protected:
    float          onStep(float u)                                override;
    void           onInitialize(const nlohmann::json& config)     override;
    nlohmann::json onSerializeJson()                        const override;
    void           onDeserializeJson(const nlohmann::json& state) override;
    int            schemaVersion()  const override { return kSchemaVersion_; }
    const char*    typeName()       const override { return "FilterSS"; }

private:
    static constexpr int kSchemaVersion_ = 1;

    void setDimension(uint8_t dim);

    MatNN phi_;
    MatN1 gamma_;
    Mat1N h_;
    Mat11 j_;
    MatN1 x_;

    uint16_t error_code_ = 0;
};

}  // namespace liteaerosim::control
