
#pragma once

#include "Control/control.hpp"
#include "Control/SISOBlock.hpp"
#include "Control/Filter.hpp"
#include "Control/FilterSS2.hpp"
#include <Eigen/Dense>


namespace Control {

// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
// template <char NUM_STATES=FILTER_MAX_STATES>
class FilterSS : public Filter
{

public:
    FilterSS() { }

    FilterSS(FilterSS &filt)
    {
        copy(filt);
    }

    void copy(FilterSS &filt);
    void copy(FilterSS2 &filt);

    // IIR filter design
    void setButterworthIIR(char order, float dt, float wn_rps);    // Butterworth low pass IIR filter design

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
    float dcGain() const;

    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, 0, NUM_STATES, NUM_STATES> MatNN;
    typedef Eigen::Matrix<float, Eigen::Dynamic,              1, 0, NUM_STATES,          1> MatN1;
    typedef Eigen::Matrix<float,              1, Eigen::Dynamic, Eigen::RowMajor,          1, NUM_STATES> Mat1N;
    typedef Eigen::Matrix<float,              1,              1, 0,          1,          1> Mat11;

    const MatNN& Phi() const {return _Phi;}
    const MatN1& Gamma() const {return _Gamma;}
    const Mat1N& H() const {return _H;}
    const Mat11& J() const {return _J;}
    const MatN1& x() const {return _x;}

    char order() const {return _Phi.rows();}

private:

    // state space realization matrices
    MatNN _Phi;
    MatN1 _Gamma;
    Mat1N _H;
    Mat11 _J;

    // state vector
    MatN1 _x;
};

}
