
#pragma once

#include "Control/SISOBlock.hpp"
#include "Control/control.hpp"
#include <Eigen/Dense>


namespace Control {


// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
// template <char NUM_STATES=FILTER_MAX_STATES>
class Filter : protected SISOBlock
{

public:

    Filter::Filter() : _errorCode(0), maxNumStates(0) {}

    void copy(Filter &filt);

    Eigen::size_t order() const;

    // step the filter
    float step(float in);

    // reset the fiter based on inputs
    void resetInput(float in);

    // Reset the filter based on outputs
    // If dc gain is zero, then the filter is
    // reset to zero regardless of argument value
    void resetOutput(float out);

    // dc gain value of the filter
    float dcGain() const;

    // retrieve the errorCode bitmask
    uint16_t errorCode() const {
        return _errorCode;
    };

protected:

    const char maxNumStates = NUM_STATES;
    uint16_t _errorCode;

};

}
