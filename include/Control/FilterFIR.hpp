
#pragma once

#include "control/SISOBlock.hpp"
#include "control/Filter.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>


namespace Control {

// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
// template <char NUM_STATES=FILTER_MAX_STATES>
class FilterFIR : public Filter
{

public:
    FilterFIR()
    {
        num << 1;
        uBuff << 0;
    }

    FilterFIR(FilterFIR &filt)
    {
        copy(filt);
    }

    void copy(FilterFIR &filt);

    Eigen::size_t order() { return num.rows() - 1; }

    // FIR filter design
    void setAverageFIR(char order);        // equal weight moving average FIR filter design
    void setExpFIR(char order, float dt, float tau); // exponential decaying weight moving average FIR filter design

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

private:

    FiltVectorXf num;
    FiltVectorXf uBuff;

};

}