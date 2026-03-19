#define _USE_MATH_DEFINES

#include <cmath>
#include "control/SISOPIDFF.hpp"
#include "math/math_util.hpp"

using namespace liteaerosim::control;

SISOPIDFF::SISOPIDFF() {}

SISOPIDFF::SISOPIDFF(const SISOPIDFF& other) {
    copy(other);
}

void SISOPIDFF::copy(const SISOPIDFF& other) {
    command_signal_                  = other.command_signal_;
    feedforward_signal_              = other.feedforward_signal_;
    measurement_signal_              = other.measurement_signal_;
    measurement_derivative_signal_   = other.measurement_derivative_signal_;
    error_signal_                    = other.error_signal_;
    output_signal_                   = other.output_signal_;
    command_unwrap_                  = other.command_unwrap_;
    measurement_unwrap_              = other.measurement_unwrap_;
    proportional_gain_               = other.proportional_gain_;
    integral_gain_                   = other.integral_gain_;
    derivative_gain_                 = other.derivative_gain_;
    feedforward_gain_                = other.feedforward_gain_;
    integrator_                      = other.integrator_;
    derivative_                      = other.derivative_;
    unwrap_inputs_                   = other.unwrap_inputs_;
}

// ---------------------------------------------------------------------------
// DynamicElement lifecycle
// ---------------------------------------------------------------------------

void SISOPIDFF::onInitialize(const nlohmann::json& /*config*/) {}

void SISOPIDFF::onReset() {
    reset(0.0f, 0.0f, 0.0f);
}

nlohmann::json SISOPIDFF::onSerializeJson() const {
    return {
        {"command_signal",                command_signal_.serializeJson()},
        {"feedforward_signal",            feedforward_signal_.serializeJson()},
        {"measurement_signal",            measurement_signal_.serializeJson()},
        {"measurement_derivative_signal", measurement_derivative_signal_.serializeJson()},
        {"error_signal",                  error_signal_.serializeJson()},
        {"output_signal",                 output_signal_.serializeJson()},
        {"command_unwrap",                command_unwrap_.serializeJson()},
        {"measurement_unwrap",            measurement_unwrap_.serializeJson()},
        {"proportional_gain",             proportional_gain_.value()},
        {"integral_gain",                 integral_gain_.value()},
        {"derivative_gain",               derivative_gain_.value()},
        {"feedforward_gain",              feedforward_gain_.value()},
        {"integrator",                    integrator_.serializeJson()},
        {"derivative",                    derivative_.serializeJson()},
        {"unwrap_inputs",                 unwrap_inputs_}
    };
}

void SISOPIDFF::onDeserializeJson(const nlohmann::json& state) {
    command_signal_.deserializeJson(state.at("command_signal"));
    feedforward_signal_.deserializeJson(state.at("feedforward_signal"));
    measurement_signal_.deserializeJson(state.at("measurement_signal"));
    measurement_derivative_signal_.deserializeJson(state.at("measurement_derivative_signal"));
    error_signal_.deserializeJson(state.at("error_signal"));
    output_signal_.deserializeJson(state.at("output_signal"));
    command_unwrap_.deserializeJson(state.at("command_unwrap"));
    measurement_unwrap_.deserializeJson(state.at("measurement_unwrap"));
    proportional_gain_.set(state.at("proportional_gain").get<float>());
    integral_gain_.set(state.at("integral_gain").get<float>());
    derivative_gain_.set(state.at("derivative_gain").get<float>());
    feedforward_gain_.set(state.at("feedforward_gain").get<float>());
    integrator_.deserializeJson(state.at("integrator"));
    derivative_.deserializeJson(state.at("derivative"));
    unwrap_inputs_ = state.at("unwrap_inputs").get<bool>();
}

// ---------------------------------------------------------------------------
// Step
// ---------------------------------------------------------------------------

float SISOPIDFF::step(float command, float measurement, float measurement_derivative) {

    float prop_prev  = proportional();
    float ff_prev    = feedForward();
    float deriv_prev = derivativeTerm();

    // optional cmd/meas unwrapping
    if (unwrap_inputs_) {
        measurement_unwrap_.step(measurement);
        command_unwrap_.step(command);
    } else {
        measurement_unwrap_.resetTo(measurement);
        command_unwrap_.resetTo(command);
    }

    command_signal_.step(command_unwrap_.out());
    measurement_signal_.step(measurement_unwrap_.out());

    float error_unfiltered = command_signal_.out() - measurement_signal_.out();

    if (unwrap_inputs_) {

        error_unfiltered = MathUtil::wrapToPi(error_unfiltered);

        // We need to support the following for angular coordinates:
        //
        // Error signal may need to be clipped to a value bound (e.g. +/- pi/4)
        //
        // Error signal may need to be rate limited and filtered, but also the command
        // may cycle past the pi boundary and appear on the other side.  When that occurs,
        // we do *not* want the rate limit and filter to slew the error through 0 but
        // rather we want it to snap to the opposite limit.
        //
        // However, while the error signal is larger than the value limits, we do
        // not want to filter and rate limiter states to wind up past the value
        // limits, because doing so would cause the filtered value to lag coming
        // off of the limits.
        //
        // Solution: detect wrap crossing events and handle those separately.
        // We want to reset errSignal when that occurs.
        // We will see the one of the following conditions on a reset event:
        // 1) wrapToPi(errorUnfiltered - errorUnfiltered_prev) is opposite sign from
        //    wrapToPi(errorUnfiltered)
        //    In this case, the unfiltered value on its own is observed going
        //    past the pi boundary.
        //
        // 2) wrapToPi(errorUnfiltered - errSignal.out()) is opposite sign from
        //    wrapToPi(errorUnfiltered)
        //    AND (both errorUnfiltered and errSignal.out() are nonzero
        //    AND errorUnfiltered and errSignal.out() are opposite sign from each other
        //    OR their wrapped difference is zero (i.e both are at the pi boundary))
        //    In this case, the unfiltered value is closer to the filtered value
        //    through the pi boundary than it is within the limited range.
        //
        // Probably detecting #2 is the only condition test needed.
        // In either case, the behavior we want to perform is to reset the
        // filtered value to the nearest value limit to the unfiltered value.
        // TODO: should we then step the filter, or should we wait until the
        // next iteration?

        if ((fabs(error_unfiltered) > 0.0f)
            && (fabs(error_signal_.out()) > 0.0f)
            && error_unfiltered * MathUtil::wrapToPi(error_signal_.out()) < 0.0f
            && (error_unfiltered * MathUtil::wrapToPi(error_unfiltered - error_signal_.out()) <= 0.0f)) {

            // The error inputs have crossed the pi boundary, reset the filter to close
            // to the nearest pi limit

            float pi_bound_nearest = (error_unfiltered < 0.0f) ? -M_PI : M_PI;

            // NOTE: one effect of this reset is that the rate limit and filtering
            // may have a jump as we cross the boundary due to loss of the information
            // of the previous error value as we jump to the nearest limit.  The size of the jump
            // will be proportional to how fast the unlimited error values are changing.
            // We will try to maintain that information by allowing the reset value to exceed
            // the pi boundary, but the value limit if enabled won't allow that.
            // In other words, say the previous filtered error value was pi - 0.01
            // but the unfiltered error value crossed to -pi + 0.05.  Then we want to reset the
            // error filter to -pi - 0.01.  The value of piBoundNearest will be -pi.
            //
            // If on the other hand the value limit is enabled at +/- pi/4, the previous filtered
            // error was pi/4 - 0.01 and the unfiltered error value has crossed to
            // -pi + 0.05.  Then piBoundNearest = -pi and we will reset to
            // -pi + wrapToPi(pi/4-0.01 - - pi) = -pi -3*pi/4 - 0.01
            // but the value limit will clip the errSignal output (and state) to -pi/4.

            error_signal_.resetToInput(pi_bound_nearest
                                       + MathUtil::wrapToPi(error_signal_.out() - pi_bound_nearest));
        }

        error_signal_.step(error_unfiltered);

        // feed forward should probably only be used for cartesian input coordinates
        feedforward_signal_.resetToInput(0.0f);

    } else {
        error_signal_.step(error_unfiltered);

        // feed forward should probably only be used for cartesian input coordinates
        feedforward_signal_.step(command_unwrap_.out());
    }

    // backsolve
    integrator_.resetTo(output() - ff_prev - prop_prev - deriv_prev);

    integrator_.step(integral_gain_ * error_signal_.out());

    measurement_derivative_signal_.step(measurement_derivative);

    output_signal_.step(feedForward() + proportional() + integral() + derivativeTerm());

    return output();
}

float SISOPIDFF::step(float command, float measurement) {
    return step(command, measurement, derivative_.step(measurement));
}

void SISOPIDFF::reset(float command, float measurement, float measurement_derivative, float output) {
    // optional cmd/meas unwrapping
    if (unwrap_inputs_) {
        measurement_unwrap_.step(measurement);
        command_unwrap_.step(command, measurement_unwrap_.out());
    } else {
        measurement_unwrap_.resetTo(measurement);
        command_unwrap_.resetTo(command);
    }

    command_signal_.resetToInput(command_unwrap_.out());
    measurement_signal_.resetToInput(measurement_unwrap_.out());
    error_signal_.resetToInput(command_signal_.out() - measurement_signal_.out());
    feedforward_signal_.resetToInput(command_unwrap_.out());

    derivative_.resetTo(measurement, measurement_derivative);

    measurement_derivative_signal_.resetToInput(derivative_.out());

    output_signal_.resetToOutput(output);

    integrator_.resetTo(output - (feedForward() + proportional() + derivativeTerm()));
}

void SISOPIDFF::reset(float command, float measurement, float output) {
    reset(command, measurement, 0.0f, output);
}
