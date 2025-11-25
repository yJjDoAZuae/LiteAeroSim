
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

    Limit valLimit;
    Limit rateLimit;

    // step the filter
    float step(float in);

    // reset the fiter based on inputs
    void resetInput(float in);

    // Reset the filter based on outputs
    // If dc gain is zero, then the filter is
    // reset to zero regardless of argument value
    void resetOutput(float out);

protected:

    void backsolve1(float inPrev, float outPrev);
    void backsolve2(float inPrev, float outPrev);
    void backsolve(float inPrev, float outPrev);

};

}
