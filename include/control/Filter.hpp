#pragma once

#include "SisoElement.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>

namespace liteaerosim::control {

/// Abstract base for all discrete filter implementations.
///
/// Extends SisoElement with filter-specific query and reset methods.
/// All concrete filters (FilterSS2, FilterSS2Clip, FilterTF, FilterTF2,
/// FilterFIR, FilterSS) derive from this class and implement the full
/// SisoElement NVI lifecycle.
class Filter : public liteaerosim::SisoElement {
public:
    constexpr static char maxNumStates = liteaerosim::kFilterMaxStates;

    /// Number of filter poles.
    virtual uint8_t order() const = 0;

    /// DC gain of the discrete filter transfer function.
    virtual float dcGain() const = 0;

    /// Reset filter state assuming the input has been at in_val for a long time.
    virtual void resetToInput(float in_val) = 0;

    /// Reset filter state to produce output value out_val.
    /// If DC gain is zero the filter resets to zero regardless of out_val.
    virtual void resetToOutput(float out_val) = 0;

    /// Bitmask of active error flags (0 = no error).
    /// See liteaerosim::control::FilterError for bit definitions.
    virtual uint16_t errorCode() const = 0;
};

} // namespace liteaerosim::control
