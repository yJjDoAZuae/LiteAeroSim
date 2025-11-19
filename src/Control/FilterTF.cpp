#include "math.h"
// #include <stdio.h>
// #include <string.h>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterTF.hpp"

static float dcTol = 1e-6;

using namespace Control;

// error_code values
// 0 : no error
// 1 : invalid vector dimension
// 2 : invalid timestep
// 3 : invalid denominator coefficients
// 4 : non-finite dc gain
// 5 : output initialization with zero dc gain

// copy implementation
// template <char NUM_STATES=FILTER_MAX_STATES>
// void SISOFilter<NUM_STATES>::copy(SISOFilter &filt)
void FilterTF::copy(FilterTF &filt)
{
    char n = (maxNumStates >= filt.order()) ? filt.order() : maxNumStates;

    den = filt.den.head(n + 1);
    num = filt.num.head(n + 1);
    uBuff = filt.uBuff.head(n + 1);
    yBuff = filt.yBuff.head(n + 1);

    // we ignore the first entry in den and set it to 1 in all cases
    den(0) = 1.0f;

    _errorCode += filt.errorCode();
}

// https://en.wikipedia.org/wiki/Butterworth_filter#Normalized_Butterworth_polynomials
void FilterTF::setButterworthIIR(char order, float dt, float wn_rps)
{

    if (order > 10 || order > maxNumStates) {
        return;
    }

    FiltVectorXf num_s;
    FiltVectorXf den_s;

    num_s.resize(order+1);
    den_s.resize(order+1);

    num_s << 1;
    float a = 1/wn_rps;
    switch (order) {
        case 0:
            // DC pass through
            den_s << 1;
            break;

        case 1:
            den_s << 1, 1;
            break;
        case 2:
            den_s << 1, 1.4142, 1;
            break;
        case 3:
            den_s << 1, 2, 2, 1;
            break;
        case 4:
            den_s << 1, 2.6131, 3.4142, 2.6131, 1;
            break;
        case 5:
            den_s << 1, 3.2361, 5.2361, 5.2361, 3.2361, 1;
            break;
        case 6:
            den_s << 1, 3.8637, 7.4641, 9.1416, 7.4641, 3.8637, 1;
            break;
        case 7:
            den_s << 1, 4.4940, 10.0978, 14.5918, 14.5918, 10.0978, 4.4940, 1;
            break;
        case 8:
            den_s << 1, 5.1258, 13.1371, 21.8462, 25.6884, 21.8462, 13.1371, 5.1258, 1;
            break;
        case 9:
            den_s << 1, 5.7588, 16.5817, 31.1634, 41.9864, 41.9864, 31.1634, 16.5817, 5.7588, 1;
            break;
        case 10:
            den_s << 1, 6.3925, 20.4317, 42.8021, 64.8824, 74.2334, 64.8824, 42.8021, 20.4317, 6.3925, 1;
            break;
    }

    // update the coefficients for wn_rps
    for (int k = 0; k < den_s.size(); k++) {
        den_s(k) *= 1/pow(wn_rps, order-k);
    }

    _errorCode += tustin_n_tf(num_s, den_s, dt, num, den);

}

void FilterTF::resetInput(float in)
{
    const float tol = 1e-6;

    uBuff.resize(order()+1);
    yBuff.resize(order()+1);
    uBuff.setZero();
    yBuff.setZero();

    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {
        uBuff << FiltVectorXf::Ones(order()+1) * in;
        yBuff << FiltVectorXf::Ones(order()+1) * in * dcGain;
    }

    _in = uBuff(0);
    _out = yBuff(0);
}

void FilterTF::resetOutput(float out)
{
    const float tol = 1e-6;

    uBuff.resize(order()+1);
    yBuff.resize(order()+1);
    uBuff.setZero();
    yBuff.setZero();

    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {
        uBuff << FiltVectorXf::Ones(order()+1) * out / dcGain;
        yBuff << FiltVectorXf::Ones(order()+1) * out;
    }

    _in = uBuff(0);
    _out = yBuff(0);
}

float FilterTF::dcGain()
{
    const float tol = 1e-6;

    // float dcGain = 1.0f;

    float numSum = num.sum();
    float denSum = den.sum();

    // We need an arbitrary finite DC gain to accommodate
    // band pass, high pass, and lead/lag filters filters
    // TODO: Filter stability test?

    // Test for infinite DC gain.
    // Pure integrators should not be implemented
    // using an ARMA filter.
    if (denSum < tol)
    {
        // non-finite DC gain
        // errorCode = 4;
        return 1.0f;
    }

    return numSum / denSum;
}

float FilterTF::step(float in)
{

    this->_in = in;
    this->_out = num(0) * in;

    // NOTE: implicit state->a.k[0] == 1 because
    // we're assigning state->y.k[0] without a coefficient

    // update the output
    for (int k = 1; k < order() + 1; k++)
    {
        // TRICKY: because we haven't rolled the buffers yet
        // the input and output buffer indices are one less
        // than the coefficient indices
        this->_out += num(k) * uBuff(k - 1);
        this->_out -= den(k) * yBuff(k - 1);
    }

    roll_buffer(yBuff, this->out());
    roll_buffer(uBuff, this->in());

    return this->out();
}
