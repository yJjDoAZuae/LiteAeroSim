#define _USE_MATH_DEFINES
#include <cmath>
// #include <stdio.h>
// #include <string.h>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterSS2Clip.hpp"

static float dcTol = 1e-6;

using namespace Control;

void FilterSS2Clip::resetInput(float in)
{
    const float tol = 1e-6;

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
        _out = valLimit.step(dcGain * in);

        if (fabs(dcGain) > tol) {
            _in = _out/dcGain;
        }

        _x << ImPhiInv * _Gamma * _in;
    } else {
        return;
    }
}

void FilterSS2Clip::resetOutput(float out)
{
    const float tol = 1e-6;

    _x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (fabs(dcGain) < tol) {
        _errorCode += FilterError::ZERO_DC_GAIN;
        return;
    }

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

        // Assumption that rateLimit limits interval includes 0
        // otherwise we would need to do a ramp reset rather than DC
        rateLimit.step(0.0f);
        _out = valLimit.step(out);
        _in = out/dcGain;

        _x << ImPhiInv * _Gamma * _in;
    } else {
        return;
    }

}

void FilterSS2Clip::backsolve(float inPrev, float outPrev)
{
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

        // y_k = H Phi^-1 (x_k - Gamma u_k-1) + J u_k-1
        // y_k+1 = H x_k + J u_k
        Mat22 A;
        A << _H*PhiInv,
            _H;

        Mat22 AInv;
        A.computeInverseWithCheck(AInv, invertible, absDeterminantThreshold);

        Mat21 b;
        b << outPrev + (_H * PhiInv * _Gamma * inPrev).value() - (_J * inPrev).value(), 
            _out - (_J * _in).value();

        if (invertible) {
            _x = AInv * b;
        } else {
            // A is uninvertible, try the psuedoinverse instead
            Mat22 ApInv;
            Mat22 ATAInv;
            (A.transpose()*A).computeInverseWithCheck(ATAInv, invertible, absDeterminantThreshold);

            if (invertible) {
                ApInv = ATAInv * A.transpose();
                _x = ApInv * b;
            }
        }
    }
}

float FilterSS2Clip::step(float in)
{

    float inPrev = _in;
    _in = in;

    float delU = in - inPrev;

    float outPrev = _out;

    // TRICKY: update the output first
    _out = (_H*_x + _J*_in).value();

    // Euler derivative computation with derivative
    // TODO: use a tustin derivative here?
    float outDot = rateLimit.step((_out - outPrev)/_dt);

    // Apply the rate limit to the output but also reimpose the value limit
    _out = valLimit.step(outPrev + outDot*_dt);

    if (valLimit.isLimited() || rateLimit.isLimited()) {
        backsolve(inPrev, outPrev);
    }

    // state update
    _x = _Phi*_x + _Gamma*_in;

    return this->out();
}
