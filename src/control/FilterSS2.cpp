#define _USE_MATH_DEFINES
#include <cmath>
// #include <stdio.h>
// #include <string.h>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterSS2.hpp"

static float dcTol = 1e-6;

using namespace Control;

// copy implementation
// template <char NUM_STATES=FILTER_MAX_STATES>
// void SISOFilter<NUM_STATES>::copy(SISOFilter &filt)
void FilterSS2::copy(FilterSS2 &filt)
{
    _Phi << filt.Phi();
    _Gamma << filt.Gamma();
    _H << filt.H();
    _J << filt.J();
    _x << filt.x();

    _errorCode = filt.errorCode();
    
    _order = filt.order();
}

void FilterSS2::setLowPassFirstIIR(float dt, float tau)
{
    // HACK: zero order hold realization
    // ydot = -1/tau y + 1/tau u
    // yk+1 - yk = (-1/tau yk + 1/tau u) * dt
    // yk+1 = (1 - dt/tau) xk + dt/tau u

    // numz.k[0] = dt/tau;
    // numz.k[1] = 0.0f;
    // denz.k[0] = 1.0f;
    // denz.k[1] = -(1-dt/tau);

    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 0.0f;
    num_s(1) = 0.0f;
    num_s(2) = 1.0f / tau;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode += tustin_1_tf(num_s, den_s, dt, 2.0f*M_PI / tau, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 1;

}

void FilterSS2::setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero)
{
    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 0.0f;  // s^2
    num_s(1) = tau_zero * wn_rps * wn_rps; // s
    num_s(2) = wn_rps * wn_rps;
    den_s(0) = 1.0f;  // s^2
    den_s(1) = 2.0f * zeta * wn_rps;  // s
    den_s(2) = wn_rps * wn_rps;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode += tustin_2_tf(num_s, den_s, dt, wn_rps, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 2;
}

void FilterSS2::setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num)
{
    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 1.0f;
    num_s(1) = 2.0f * zeta_num * wn_rps;
    num_s(2) = wn_rps * wn_rps;
    den_s(0) = 1.0f;
    den_s(1) = 2.0f * zeta_den * wn_rps;
    den_s(2) = wn_rps * wn_rps;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode += tustin_2_tf(num_s, den_s, dt, wn_rps, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 2;
}

void FilterSS2::setHighPassFirstIIR(float dt, float tau)
{
    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 0.0f;
    num_s(1) = 1.0f / tau;
    num_s(2) = 0.0f;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode = tustin_2_tf(num_s, den_s, dt, 2.0f*M_PI / tau, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 1;
}

void FilterSS2::setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero)
{
    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 1.0f;  // s^2
    num_s(1) = c_zero * 2.0f * zeta * wn_rps;  // s
    num_s(2) = 0.0f;
    den_s(0) = 1.0f;  // s^2
    den_s(1) = 2.0f * zeta * wn_rps;  // s
    den_s(2) = wn_rps * wn_rps;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode += tustin_2_tf(num_s, den_s, dt, wn_rps, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 2;
}

void FilterSS2::setDerivIIR(float dt, float tau)
{
    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 0.0f;
    num_s(1) = 1.0f / tau;
    num_s(2) = 0.0f;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode += tustin_2_tf(num_s, den_s, dt, 2.0f*M_PI / tau, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 1;
}

void FilterSS2::resetInput(float in)
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

void FilterSS2::resetOutput(float out)
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

float FilterSS2::dcGain() const
{
    // float dcGain = 1.0f;

    // We need an arbitrary finite DC gain to accommodate
    // band pass, high pass, and lead/lag filters filters
    // TODO: Filter stability test?

    // Test for infinite DC gain.
    // Pure integrators should not be implemented
    // using an ARMA filter.

    Mat22 ImPhiInv;
    bool invertible = false;
    float absDeterminantThreshold = 1e-4;

    (Mat22::Identity() - _Phi).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

    if (!invertible) {
        // _errorCode += FilterError::INFINITE_DC_GAIN;
        return 1.0f;
    }

    return (_H*ImPhiInv*_Gamma + _J).value();
}

Mat22 FilterSS2::controlGrammian() const
{
    Mat22 C(Mat22::Zero(2,2));

    for (int k = 0; k<2; k++) {
        C(Eigen::all, k) << Mat21(_Phi.pow(k) * _Gamma);
    }

    return C;
}

Mat22 FilterSS2::observeGrammian() const
{
    Mat22 C(Mat22::Zero(2,2));

    for (int k = 0; k<2; k++) {
        C(k, Eigen::all) << Mat12(_H * _Phi.pow(k));
    }

    return C;
}

float FilterSS2::step(float in)
{

    this->_in = in;

    // TRICKY: update the output first
    _out = (_H*_x + _J*in).value();

    // now update the state
    _x = _Phi*_x + _Gamma*in;

    return this->out();
}
