
#pragma once

#include "control/SISOBlock.hpp"
#include "control/Filter.hpp"
#include "control/FilterSS2.hpp"
#include "control/Limit.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>  // for Matrix::pow()

namespace Control {

// template <char NUM_STATES=FILTER_MAX_STATES>

class FilterSS2Clip : public FilterSS2 {

public:
    FilterSS2Clip()
    { 
        FilterSS2::FilterSS2();
    }

    FilterSS2Clip(FilterSS2Clip &filt)
    {
        copy(filt);
    }

    void copy(FilterSS2Clip &filt);

    // step the filter
    float step(float in);

    // reset the fiter based on inputs
    void resetInput(float in);

    // Reset the filter based on outputs
    // If dc gain is zero, then the filter is
    // reset to zero regardless of argument value
    void resetOutput(float out);

protected:

    // 2nd order state space realization matrices
    Mat22 _Phi;
    Mat21 _Gamma;
    Mat12 _H;
    Mat11 _J;

    // 2nd order state vector
    Mat21 _x;

    Limit valLimit;
    Limit rateLimit;

    uint8_t _order;

};

}
