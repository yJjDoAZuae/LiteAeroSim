
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

    void copy(Filter &filt);

    char order();

    // step the filter
    float step(float in);

    // reset the fiter based on inputs
    void resetInput(float in);

    // Reset the filter based on outputs
    // If dc gain is zero, then the filter is
    // reset to zero regardless of argument value
    void resetOutput(float out);

    // dc gain value of the filter
    float dcGain();

    // retrieve the last errorCode generated
    int lastError() { return errorCode; };

protected:

    FilterError errorCode;

};

}