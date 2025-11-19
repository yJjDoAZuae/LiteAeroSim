
#pragma once

#include "Control/control.hpp"
#include "Control/SISOBlock.hpp"
#include "Control/Filter.hpp"
#include "Control/FilterSS2.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>  // for Matrix::pow()

namespace Control {

// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
// template <char NUM_STATES=FILTER_MAX_STATES>
class FilterSS : public Filter
{

public:
    FilterSS()
    {
        _Phi.resize(0,0);
        _Gamma.resize(0,1);
        _H.resize(1,0);
        _J.setOnes();
        _x.resize(0,1);
    }


    FilterSS(FilterSS &filt)
    {
        copy(filt);
    }

    void copy(FilterSS &filt);
    void copy(FilterSS2 &filt);

    // IIR filter design
    void setButterworthIIR(char order, float dt, float wn_rps);    // Butterworth low pass IIR filter design

    Eigen::size_t order() const;

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

    MatNN Phi() const {return _Phi;}
    MatN1 Gamma() const {return _Gamma;}
    Mat1N H() const {return _H;}
    Mat11 J() const {return _J;}
    MatN1 x() const {return _x;}

    MatNN controlGrammian() const;
    MatNN observeGrammian() const;

protected:

    void setDimension(char dim);

    // state space realization matrices
    MatNN _Phi;
    MatN1 _Gamma;
    Mat1N _H;
    Mat11 _J;

    // state vector
    MatN1 _x;
};

}
