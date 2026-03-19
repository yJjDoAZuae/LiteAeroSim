#pragma once

#include "DynamicElement.hpp"
#include "control/control.hpp"
#include "control/Gain.hpp"
#include "control/FilterSS2Clip.hpp"
#include "control/Integrator.hpp"
#include "control/Derivative.hpp"
#include "control/Unwrap.hpp"

namespace liteaerosim::control {

/// Single-input single-output PID with feedforward.
///
/// Derives from DynamicElement and implements the full lifecycle:
/// initialize → reset → step×N → serializeJson / deserializeJson.
///
/// The six FilterSS2Clip members, Integrator, Derivative, and two Unwrap
/// members each carry their own serializable state. SISOPIDFF coordinates
/// their lifecycle and exposes a composite snapshot.
class SISOPIDFF : public liteaerosim::DynamicElement {
public:
    SISOPIDFF();
    SISOPIDFF(const SISOPIDFF& other);
    void copy(const SISOPIDFF& other);

    /// Step with internally computed measurement derivative.
    float step(float command, float measurement);

    /// Step with externally provided measurement derivative.
    float step(float command, float measurement, float measurement_derivative);

    /// Reset based on command, measurement, and desired output.
    void reset(float command, float measurement, float output);

    /// Reset based on command, measurement, measurement derivative, and desired output.
    void reset(float command, float measurement, float measurement_derivative, float output);

    // --- Read accessors ---
    float output()                const { return output_signal_.out(); }
    float command()               const { return command_signal_.in(); }
    float measurement()           const { return measurement_signal_.in(); }
    float measurementDerivative() const { return measurement_derivative_signal_.in(); }
    float error()                 const { return error_signal_.in(); }
    float feedForward()           const { return feedforward_gain_ * feedforward_signal_.out(); }
    float proportional()          const { return proportional_gain_ * error_signal_.out(); }
    float derivativeTerm()        const { return derivative_gain_ * measurement_derivative_signal_.out(); }
    float integral()              const { return integrator_.out(); }

    /// Enable or disable angle-unwrapping of command and measurement inputs.
    void setUnwrapInputs(bool value) { unwrap_inputs_ = value; }

    /// Mutable reference to the integrator (e.g., for warm-start calls like resetTo).
    Integrator& integrator() { return integrator_; }

    // --- Mutable sub-element references for configuration ---
    FilterSS2Clip& commandSignal()               { return command_signal_; }
    FilterSS2Clip& feedforwardSignal()           { return feedforward_signal_; }
    FilterSS2Clip& measurementSignal()           { return measurement_signal_; }
    FilterSS2Clip& measurementDerivativeSignal() { return measurement_derivative_signal_; }
    FilterSS2Clip& errorSignal()                 { return error_signal_; }
    FilterSS2Clip& outputSignal()                { return output_signal_; }

    Gain<float, 3>& proportionalGain()  { return proportional_gain_; }
    Gain<float, 3>& integralGain()      { return integral_gain_; }
    Gain<float, 3>& derivativeGain()    { return derivative_gain_; }
    Gain<float, 3>& feedforwardGain()   { return feedforward_gain_; }

protected:
    void           onInitialize(const nlohmann::json& config) override;
    void           onReset()                                   override;
    nlohmann::json onSerializeJson()                   const   override;
    void           onDeserializeJson(const nlohmann::json& state) override;
    int            schemaVersion()                     const   override { return 1; }
    const char*    typeName()                          const   override { return "SISOPIDFF"; }

private:
    FilterSS2Clip command_signal_;
    FilterSS2Clip feedforward_signal_;
    FilterSS2Clip measurement_signal_;
    FilterSS2Clip measurement_derivative_signal_;
    FilterSS2Clip error_signal_;  ///< For angular coords, valLimit bounds must be opposite sign if enabled.
    FilterSS2Clip output_signal_;

    Unwrap command_unwrap_;
    Unwrap measurement_unwrap_;

    Gain<float, 3> proportional_gain_;
    Gain<float, 3> integral_gain_;
    Gain<float, 3> derivative_gain_;
    Gain<float, 3> feedforward_gain_;

    Integrator integrator_;
    Derivative derivative_;

    bool unwrap_inputs_ = false;
};

}  // namespace liteaerosim::control
