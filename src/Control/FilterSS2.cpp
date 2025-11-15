#include "math.h"
// #include <stdio.h>
// #include <string.h>

#include "Control/control.hpp"
#include "Control/filter_realizations.hpp"
#include "Control/FilterSS2.hpp"

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
void FilterSS2::copy(FilterSS2 &filt)
{
    Phi << filt.Phi;
    Gamma << filt.Gamma;
    H << filt.H;
    J << filt.J;
    x << filt.x;

    errorCode = filt.errorCode;
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

    Eigen::Vector<float, 3> num_s;
    Eigen::Vector<float, 3> den_s;

    num_s(0) = 0.0f;
    num_s(1) = 0.0f;
    num_s(2) = 1.0f / tau;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    Eigen::Vector<float, 3> num_z;
    Eigen::Vector<float, 3> den_z;

    errorCode = tustin_2_tf(num_s, den_s, dt, num_z, den_z);

    tf2ss(num_z, den_z, Phi, Gamma, H, J);

}

void FilterSS2::setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero)
{
    Eigen::Vector<float, 3> num_s;
    Eigen::Vector<float, 3> den_s;

    num_s(0) = 0.0f;  // s^2
    num_s(1) = tau_zero * wn_rps * wn_rps; // s
    num_s(2) = wn_rps * wn_rps;
    den_s(0) = 1.0f;  // s^2
    den_s(1) = 2.0f * zeta * wn_rps;  // s
    den_s(2) = wn_rps * wn_rps;

    Eigen::Vector<float, 3> num_z;
    Eigen::Vector<float, 3> den_z;

    errorCode = tustin_2_tf(num_s, den_s, dt, num_z, num_z);

    tf2ss(num_z, den_z, Phi, Gamma, H, J);
}

void FilterSS2::setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num)
{
    Eigen::Vector<float, 3> num_s;
    Eigen::Vector<float, 3> den_s;

    num_s(0) = 1.0f;
    num_s(1) = 2.0f * zeta_num * wn_rps;
    num_s(2) = wn_rps * wn_rps;
    den_s(0) = 1.0f;
    den_s(1) = 2.0f * zeta_den * wn_rps;
    den_s(2) = wn_rps * wn_rps;

    Eigen::Vector<float, 3> num_z;
    Eigen::Vector<float, 3> den_z;

    errorCode = tustin_2_tf(num_s, den_s, dt, num_z, den_z);

    tf2ss(num_z, den_z, Phi, Gamma, H, J);
}

void FilterSS2::setHighPassFirstIIR(float dt, float tau)
{
    Eigen::Vector<float, 3> num_s;
    Eigen::Vector<float, 3> den_s;

    num_s(0) = 0.0f;
    num_s(1) = 1.0f / tau;
    num_s(2) = 0.0f;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    Eigen::Vector<float, 3> num_z;
    Eigen::Vector<float, 3> den_z;

    errorCode = tustin_2_tf(num_s, den_s, dt, num_z, den_z);

    tf2ss(num_z, den_z, Phi, Gamma, H, J);
}

void FilterSS2::setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero)
{
    Eigen::Vector<float, 3> num_s;
    Eigen::Vector<float, 3> den_s;

    num_s(0) = 1.0f;  // s^2
    num_s(1) = c_zero * 2.0f * zeta * wn_rps;  // s
    num_s(2) = 0.0f;
    den_s(0) = 1.0f;  // s^2
    den_s(1) = 2.0f * zeta * wn_rps;  // s
    den_s(2) = wn_rps * wn_rps;

    Eigen::Vector<float, 3> num_z;
    Eigen::Vector<float, 3> den_z;

    errorCode = tustin_2_tf(num_s, den_s, dt, num_z, den_z);

    tf2ss(num_z, den_z, Phi, Gamma, H, J);
}

void FilterSS2::setDerivIIR(float dt, float tau)
{
    Eigen::Vector<float, 3> num_s;
    Eigen::Vector<float, 3> den_s;

    num_s(0) = 0.0f;
    num_s(1) = 1.0f / tau;
    num_s(2) = 0.0f;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    Eigen::Vector<float, 3> num_z;
    Eigen::Vector<float, 3> den_z;

    errorCode = tustin_2_tf(num_s, den_s, dt, num_z, den_z);

    tf2ss(num_z, den_z, Phi, Gamma, H, J);
}

void FilterSS2::resetInput(float in)
{
    x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (lastError() == 0)
    {

        Eigen::Matrix<float,2,2> ImPhiInv;
        bool invertible = false;
        float absDeterminantThreshold = 1e-4;

        Eigen::Inverse(Eigen::Matrix<float,2,2>::Identity() - Phi).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

        if (!invertible) {
            errorCode = FilterError::UNSTABLE;
            return;
        }

        _in = in;
        _out = dcGain * in;

        x << ImPhiInv * Gamma * _in;
    } else {
        return;
    }
}

void FilterSS2::resetOutput(float out)
{
    x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (lastError() == 0)
    {

        Eigen::Matrix<float,2,2> ImPhiInv;
        bool invertible = false;
        float absDeterminantThreshold = 1e-4;

        Eigen::Inverse(Eigen::Matrix<float,2,2>::Identity() - Phi).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

        if (!invertible) {
            errorCode = FilterError::UNSTABLE;
            return;
        }

        _out = out;
        _in = out/dcGain;

        x << ImPhiInv * Gamma * _in;
    } else {
        return;
    }

}

float FilterSS2::dcGain()
{
    float dcGain = 1.0f;

    // We need an arbitrary finite DC gain to accommodate
    // band pass, high pass, and lead/lag filters filters
    // TODO: Filter stability test?

    // Test for infinite DC gain.
    // Pure integrators should not be implemented
    // using an ARMA filter.

    Eigen::Matrix<float,2,2> ImPhiInv;
    bool invertible = false;
    float absDeterminantThreshold = 1e-4;

    Eigen::Inverse(Eigen::Matrix<float,2,2>::Identity() - Phi).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

    if (!invertible) {
        errorCode = FilterError::INFINITE_DC_GAIN;
        return dcGain;
    }

    dcGain = (H*ImPhiInv*Gamma + J).value();

    return dcGain;
}

float FilterSS2::step(float in)
{

    this->_in = in;

    // TRICKY: update the output first
    _out = (H*x + J*in).value();

    // now update the state
    x = Phi*x + Gamma*in;

    return this->out();
}
