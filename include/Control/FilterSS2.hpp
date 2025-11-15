
#pragma once

#include "Control/SISOBlock.hpp"
#include "Control/Filter.hpp"
#include "Control/control.hpp"
#include <Eigen/Dense>

namespace Control {

// template <char NUM_STATES=FILTER_MAX_STATES>
typedef Eigen::Matrix<float, Eigen::Dynamic, 1, 0, NUM_STATES + 1, 1> FiltVectorXf;

class FilterSS2 : Filter {

public:
    FilterSS2() : errorCode(FilterError::NONE)
    {
        Phi.setZero();
        Gamma.setZero();
        H.setZero();
        J.setZero();
        x.setZero();
    }

    FilterSS2(FilterSS2 &filt) : errorCode(FilterError::NONE)
    {
        copy(filt);
    }

    void copy(FilterSS2 &filt);

    // IIR filter design
    void setLowPassFirstIIR(float dt, float tau);                  // first order low pass filter design
    void setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero);  // second order low pass filter design
    void setHighPassFirstIIR(float dt, float tau);                 // first order high pass filter design
    void setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero); // second order high pass filter design
    void setDerivIIR(float dt, float tau);                         // first order derivative + low pass filter design
    void setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num);    // second order notch filter design

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

    FilterError errorCode=FilterError::NONE;

    // 2nd order state space realization matrices
    Eigen::Matrix<float, 2, 2> Phi;
    Eigen::Matrix<float, 2, 1> Gamma;
    Eigen::Matrix<float, 1, 2> H;
    Eigen::Matrix<float, 1, 1> J;

    // 2nd order state vector
    Eigen::Vector<float, 2> x;

};

}
