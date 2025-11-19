
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
class FilterTF : public Filter
{

public:
    FilterTF()
    {
        num << 1;
        den << 1;
        uBuff << 0;
        yBuff << 0;
    }

    FilterTF(FilterTF &filt)
    {
        copy(filt);
    }

    void copy(FilterTF &filt);

    // IIR filter design
    void setButterworthIIR(char order, float dt, float wn_rps);    // Butterworth low pass IIR filter design

    Eigen::size_t order() { return den.rows() - 1; }

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
    FiltVectorXf den;

    FiltVectorXf uBuff;
    FiltVectorXf yBuff;
};

}
