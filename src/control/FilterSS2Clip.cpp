#define _USE_MATH_DEFINES
#include <cmath>
// #include <stdio.h>
// #include <string.h>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterSS2Clip.hpp"

static float dcTol = 1e-6;

using namespace Control;

// copy implementation
// template <char NUM_STATES=FILTER_MAX_STATES>
// void SISOFilter<NUM_STATES>::copy(SISOFilter &filt)
void FilterSS2Clip::copy(FilterSS2Clip &filt)
{
    _Phi << filt.Phi();
    _Gamma << filt.Gamma();
    _H << filt.H();
    _J << filt.J();
    _x << filt.x();

    _errorCode = filt.errorCode();
    
    _order = filt.order();

    valLimit = filt.valLimit;
    rateLimit = filt.rateLimit;
}

void FilterSS2Clip::resetInput(float in)
{
    _x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {

        Mat22 ImPhiInv;
        bool invertible = false;
        float absDeterminantThreshold = 1e-4;

        (Mat22::Identity() - _Phi).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

        if (!invertible) {
            _errorCode += FilterError::UNSTABLE;
            return;
        }

        _in = in;
        _out = dcGain * in;

        _x << ImPhiInv * _Gamma * _in;
    } else {
        return;
    }
}

void FilterSS2Clip::resetOutput(float out)
{
    _x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {

        Mat22 ImPhiInv;
        bool invertible = false;
        float absDeterminantThreshold = 1e-4;

        (Mat22::Identity() - _Phi).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

        if (!invertible) {
            _errorCode += FilterError::UNSTABLE;
            return;
        }

        _out = out;
        _in = out/dcGain;

        _x << ImPhiInv * _Gamma * _in;
    } else {
        return;
    }

}

float FilterSS2Clip::step(float in)
{

    float inPrev = _in;
    this->_in = in;

    float delU = in - inPrev;

    float outPrev = _out;

    // TRICKY: update the output first
    _out = valLimit.step((_H*_x + _J*in).value());

    // Euler derivative computation with derivative
    float outDot = rateLimit.step((_out - outPrev)/_dt);

    // Apply the rate limit to the output but also reimpose the value limit
    Limit valLimit2 = valLimit;
    _out = valLimit2.step(outPrev + outDot*_dt);

    // Backsolve to correct the state based on the limited values.
    // We are correcting the *previous* iteration's state here
    // to make it consistent with the output value we just calculated.
    Mat22 PhiInv;
    bool invertible = false;
    const float absDeterminantThreshold = 1e-4;

    // _Phi should be invertible for a stable filter, but if it's not
    // we'll just skip backsolving
    _Phi.computeInverseWithCheck(PhiInv, invertible, absDeterminantThreshold);

    if (invertible) {
        Mat22 A;
        A << _H, _H*_Phi.inverse();

        Mat22 AInv;
        A.computeInverseWithCheck(AInv, invertible, absDeterminantThreshold);

        Mat21 b;
        b << _out - (_J * in).value(), outPrev + (_H * PhiInv * _Gamma * inPrev).value() - (_J * inPrev).value();

        if (invertible) {
            _x = AInv * b;
        } else {
            // A is uninvertible, use the psuedoinverse instead
            Mat22 ApInv;
            ApInv = (A.transpose()*A).inverse() * A.transpose();
            
            _x = ApInv * b;
        }
    }

    // state update
    _x = _Phi*_x + _Gamma*in;

    return this->out();
}
