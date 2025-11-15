
#pragma once

#include "Control/SISOBlock.hpp"
#include "Control/Filter.hpp"
#include "Control/control.hpp"
#include <Eigen/Dense>


namespace Control {


// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
// template <char NUM_STATES=FILTER_MAX_STATES>
class FilterTF : Filter
{

public:
    FilterTF() : errorCode(0), maxNumStates(NUM_STATES)
    {
        num << 1;
        den << 1;
        uBuff << 0;
        yBuff << 0;
    }

    FilterTF(FilterTF &filt) : errorCode(0), maxNumStates(NUM_STATES)
    {
        copy(filt);
    }

    void copy(FilterTF &filt);

    // IIR filter design
    void setLowPassFirstIIR(float dt, float tau);                  // first order low pass filter design
    void setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero);  // second order low pass filter design
    void setHighPassFirstIIR(float dt, float tau);                 // first order high pass filter design
    void setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero); // second order high pass filter design
    void setDerivIIR(float dt, float tau);                         // first order derivative + low pass filter design
    void setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num);    // second order notch filter design
    void setButterworthIIR(char order, float dt, float wn_rps);    // Butterworth low pass IIR filter design

    char order() { return den.rows() - 1; }

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

    // retrieve the last errorCode generated
    int lastError() { return errorCode; };

private:

    const char maxNumStates;

    int errorCode=0;

    FiltVectorXf num;
    FiltVectorXf den;

    FiltVectorXf uBuff;
    FiltVectorXf yBuff;


    // Tustin discrete IIR filter realization
    void tustin_first();
    void tustin_second();
};

}
