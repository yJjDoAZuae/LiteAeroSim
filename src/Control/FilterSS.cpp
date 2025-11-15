
#include "math.h"
// #include <stdio.h>
// #include <string.h>

#include "Control/control.hpp"
#include "Control/filter_realizations.hpp"
#include "Control/FilterSS.hpp"

static float dcTol = 1e-6;

using namespace Control;


void FilterSS::copy(FilterSS &filt)
{
    _Phi << filt.Phi();
    _Gamma << filt.Gamma();
    _H << filt.H();
    _J << filt.J();
    _x << filt.x();

    _errorCode = filt.errorCode();
}

void FilterSS::copy(FilterSS2 &filt)
{
    _Phi << filt.Phi();
    _Gamma << filt.Gamma();
    _H << filt.H();
    _J << filt.J();
    _x << filt.x();

    _errorCode = filt.errorCode();
}


// https://en.wikipedia.org/wiki/Butterworth_filter#Normalized_Butterworth_polynomials
void FilterSS::setButterworthIIR(char order, float dt, float wn_rps)
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

    FiltVectorXf num_z;
    FiltVectorXf den_z;

    _errorCode += tustin_n_tf(num_s, den_s, dt, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

}

void FilterSS::resetInput(float in)
{
    _x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {

        char n = order();

        MatNN ImPhiInv(MatNN::Zero(n,n));
        bool invertible = false;
        float absDeterminantThreshold = 1e-4;

        MatNN ImPhi(MatNN::Identity(n,n) - _Phi);
        Eigen::FullPivLU<MatNN> LU(ImPhi);
        invertible = LU.isInvertible();

        if (!invertible) {
            _errorCode += FilterError::UNSTABLE;
            return;
        }

        ImPhiInv << LU.inverse();

        _in = in;
        _out = dcGain * in;

        _x << ImPhiInv * _Gamma * _in;
    } else {
        return;
    }
}

void FilterSS::resetOutput(float out)
{
    _x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {

        char n = order();

        MatNN ImPhiInv(MatNN::Zero(n,n));
        bool invertible = false;
        float absDeterminantThreshold = 1e-4;

        MatNN ImPhi(MatNN::Identity(n,n) - _Phi);
        Eigen::FullPivLU<MatNN> LU(ImPhi);
        invertible = LU.isInvertible();

        if (!invertible) {
            _errorCode += FilterError::UNSTABLE;
            return;
        }

        ImPhiInv << LU.inverse();

        _out = out;
        _in = out/dcGain;

        _x << ImPhiInv * _Gamma * _in;
    } else {
        return;
    }
}

float FilterSS::dcGain() const
{
    // We need an arbitrary finite DC gain to accommodate
    // band pass, high pass, and lead/lag filters filters
    // TODO: Filter stability test?

    // Test for infinite DC gain.
    // Pure integrators should not be implemented
    // using an ARMA filter.

    char n = order();

    MatNN ImPhiInv(MatNN::Zero(n,n));
    bool invertible = false;
    float absDeterminantThreshold = 1e-4;

    MatNN ImPhi(MatNN::Identity(n,n) - _Phi);
    Eigen::FullPivLU<MatNN> LU(ImPhi);
    invertible = LU.isInvertible();

    if (!invertible) {
        //_errorCode += FilterError::UNSTABLE;
        return 1.0f;
    }

    ImPhiInv << LU.inverse();

    return (_H*ImPhiInv*_Gamma + _J).value();
}

float FilterSS::step(float in)
{
    this->_in = in;

    // TRICKY: update the output first
    _out = (_H*_x + _J*in).value();

    // now update the state
    _x = _Phi*_x + _Gamma*in;

    return this->out();
}
