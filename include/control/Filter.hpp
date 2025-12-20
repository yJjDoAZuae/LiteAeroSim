
#pragma once

#include "control/SISOBlock.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>


namespace Control {


// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
// template <char NUM_STATES=FILTER_MAX_STATES>
class Filter : public SISOBlock
{

public:

    constexpr static char maxNumStates = NUM_STATES;

    virtual uint8_t order() const=0;

    // reset the fiter based on inputs
    virtual void resetInput(float in)=0;

    // Reset the filter based on outputs
    // If dc gain is zero, then the filter is
    // reset to zero regardless of argument value
    virtual void resetOutput(float out)=0;

    // dc gain value of the filter
    virtual float dcGain() const=0;

    // retrieve the errorCode bitmask
    virtual uint16_t errorCode() const=0;

};

}
