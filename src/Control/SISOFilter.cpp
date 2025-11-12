#include "math.h"
// #include <stdio.h>
// #include <string.h>

#include "Control/SISOFilter.hpp"

static float dcTol = 1e-6;

using namespace Control;

int tustin_1(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);
int tustin_2(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz);
FiltVectorXf left_resize(const FiltVectorXf &in, char len);
FiltVectorXf right_resize(const FiltVectorXf &in, char len);
void roll_buffer(FiltVectorXf &buff, float u);

// error_code values
// 0 : no error
// 1 : invalid timestep
// 2 : invalid vector dimension
// 3 : invalid denominator coefficients
// 4 : non-finite dc gain

// copy implementation
// template <char NUM_STATES=FILTER_MAX_STATES>
// void SISOFilter<NUM_STATES>::copy(SISOFilter &filt)
void SISOFilter::copy(SISOFilter &filt)
{
    char n = (NUM_STATES >= filt.order()) ? filt.order() : NUM_STATES;

    den = filt.den.head(n + 1);
    num = filt.num.head(n + 1);
    uBuff = filt.uBuff.head(n + 1);
    yBuff = filt.yBuff.head(n + 1);

    // we ignore the first entry in den and set it to 1 in all cases
    den(0) = 1.0f;
}

int tustin_1(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz)
{
    const float tol = 1.0e-6;

    numz << 1.0f;
    denz << 1.0f;

    if (num.size() < 1 || den.size() < 2) {
        return 1;
    }

    if (dt < tol) {
        return 2;
    }

    float K = 2.0f / dt;

    float coeff_denom = 1.0f;
    coeff_denom = den(0) * K + den(1);

    // printf("tustin_1: den->k[0] = %0.3f, den->k[1] = %0.3f, K = %0.3f\n", den->k[0], den->k[1], K);

    if (fabs(coeff_denom) < tol)
    {
        return 3;
    }

    // left pad or left truncate the numerator if num.size() != 2
    FiltVectorXf tmp_num = left_resize(num, 2);

    numz.resize(2);
    numz(0) = (tmp_num(0) * K + tmp_num(1)) / coeff_denom;
    numz(1) = (-tmp_num(0) * K + tmp_num(1)) / coeff_denom;

    denz.resize(2);
    denz(0) = 1.0f;
    denz(1) = (-den(0) * K + den(1)) / coeff_denom;

    return 0;
}

int tustin_2(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz)
{
    const float tol = 1.0e-6;

    numz << 1.0f;
    denz << 1.0f;

    if (num.size() < 1 || den.size() < 3) {
        return 1;
    }

    if (dt < tol) {
        return 2;
    }

    float K = 2.0f / dt;
    float coeff_denom = den(0) * K * K + den(1) * K + den(2);

    if (fabs(coeff_denom) < tol)
    {
        return 3;
    }

     // left pad or left truncate the numerator if num.size() != 3
    FiltVectorXf tmp_num = left_resize(num, 3);
    numz.resize(3);
    numz(0) = (tmp_num(0)*K*K + tmp_num(1)*K + tmp_num(2)) / coeff_denom;
    numz(1) = (-2.0*tmp_num(0)*K*K       + 2.0*tmp_num(2)) / coeff_denom;
    numz(2) = (tmp_num(0)*K*K - tmp_num(1)*K + tmp_num(2)) / coeff_denom;

    denz.resize(3);
    denz(0) = 1.0f;
    denz(1) = (-2.0*den(0)*K*K       + 2.0*den(2)) / coeff_denom;
    denz(2) = (den(0)*K*K - den(1)*K + den(2)) / coeff_denom;

    return 0;
}

void SISOFilter::setLowPassFirstIIR(float dt, float tau)
{
    // HACK: zero order hold realization
    // ydot = -1/tau y + 1/tau u
    // yk+1 - yk = (-1/tau yk + 1/tau u) * dt
    // yk+1 = (1 - dt/tau) xk + dt/tau u

    // numz.k[0] = dt/tau;
    // numz.k[1] = 0.0f;
    // denz.k[0] = 1.0f;
    // denz.k[1] = -(1-dt/tau);

    FiltVectorXf num_s;
    FiltVectorXf den_s;

    num_s.resize(2);
    den_s.resize(2);

    num_s(0) = 0.0f;
    num_s(1) = 1.0f / tau;
    den_s(0) = 1.0f;
    den_s(1) = 1.0f / tau;

    errorCode = tustin_1(num_s, den_s, dt, num, den);
}

void SISOFilter::setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero)
{
    FiltVectorXf num_s;
    FiltVectorXf den_s;

    num_s.resize(3);
    den_s.resize(3);

    num_s(0) = 0.0f;  // s^2
    num_s(1) = tau_zero * wn_rps * wn_rps; // s
    num_s(2) = wn_rps * wn_rps;
    den_s(0) = 1.0f;  // s^2
    den_s(1) = 2.0f * zeta * wn_rps;  // s
    den_s(2) = wn_rps * wn_rps;

    errorCode = tustin_2(num_s, den_s, dt, num, den);
}

void SISOFilter::setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num)
{
    FiltVectorXf num_s;
    FiltVectorXf den_s;

    num_s.resize(3);
    den_s.resize(3);

    num_s(0) = 1.0f;
    num_s(1) = 2.0f * zeta_num * wn_rps;
    num_s(2) = wn_rps * wn_rps;
    den_s(0) = 1.0f;
    den_s(1) = 2.0f * zeta_den * wn_rps;
    den_s(2) = wn_rps * wn_rps;

    errorCode = tustin_2(num_s, den_s, dt, num, den);
}

void SISOFilter::setHighPassFirstIIR(float dt, float tau)
{
    FiltVectorXf num_s;
    FiltVectorXf den_s;

    num_s.resize(2);
    den_s.resize(2);

    num_s(0) = 1.0f / tau;
    num_s(1) = 0.0f;
    den_s(0) = 1.0f;
    den_s(1) = 1.0f / tau;

    errorCode = tustin_1(num_s, den_s, dt, num, den);
}

void SISOFilter::setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero)
{
    FiltVectorXf num_s;
    FiltVectorXf den_s;

    num_s.resize(3);
    den_s.resize(3);

    num_s(0) = 1.0f;  // s^2
    num_s(1) = c_zero * 2.0f * zeta * wn_rps;  // s
    num_s(2) = 0.0f;
    den_s(0) = 1.0f;  // s^2
    den_s(1) = 2.0f * zeta * wn_rps;  // s
    den_s(2) = wn_rps * wn_rps;

    errorCode = tustin_2(num_s, den_s, dt, num, den);
}

void SISOFilter::setDerivIIR(float dt, float tau)
{
    FiltVectorXf num_s;
    FiltVectorXf den_s;

    num_s.resize(2);
    den_s.resize(2);

    num_s(0) = 1.0f / tau;
    num_s(1) = 0.0f;
    den_s(0) = 1.0f;
    den_s(1) = 1.0f / tau;

    errorCode = tustin_1(num_s, den_s, dt, num, den);
}

void SISOFilter::setAverageFIR(char order)
{
    num = FiltVectorXf::Ones(order+1)*(1.0f/(order+1.0f));
    den = FiltVectorXf::Zero(order+1);
    uBuff = FiltVectorXf::Zero(order+1);
    yBuff = FiltVectorXf::Zero(order+1);

    den(0) = 1.0f;
}

void SISOFilter::setExpFIR(char order, float dt, float tau)
{
    num = FiltVectorXf::Zero(order+1);
    den = FiltVectorXf::Zero(order+1);
    uBuff = FiltVectorXf::Zero(order+1);
    yBuff = FiltVectorXf::Zero(order+1);

    den(0) = 1.0f;

    for (int k=0; k<this->order()+1; k++) {
        num(k) = exp(-k*dt/tau);
    }
    float sumNum = num.sum();
    num *= 1.0f/sumNum;  // normalize to unity dc gain
}

void SISOFilter::resetInput(float in)
{
    const float tol = 1e-6;

    uBuff.resize(order()+1);
    yBuff.resize(order()+1);
    uBuff.setZero();
    yBuff.setZero();

    float dcGain = this->dcGain();

    if (lastError() == 0)
    {
        uBuff << FiltVectorXf::Ones(order()+1) * in;
        yBuff << FiltVectorXf::Ones(order()+1) * in * dcGain;
    }
}

void SISOFilter::resetOutput(float out)
{
    const float tol = 1e-6;

    uBuff.resize(order()+1);
    yBuff.resize(order()+1);
    uBuff.setZero();
    yBuff.setZero();

    float dcGain = this->dcGain();

    if (lastError() == 0)
    {
        uBuff << FiltVectorXf::Ones(order()+1) * out / dcGain;
        yBuff << FiltVectorXf::Ones(order()+1) * out;
    }
}

float SISOFilter::dcGain()
{
    const float tol = 1e-6;

    float dcGain = 1.0f;

    // TRICKY: potential access to uninitialized data
    // for zeroth order (static gain) case.
    float numsum = 0.0f;
    float densum = 0.0f;

    for (int k = 0; k < order() + 1; k++)
    {
        numsum += num(k);
        densum += den(k);
    }

    // We need an arbitrary finite DC gain to accommodate
    // band pass, high pass, and lead/lag filters filters
    // TODO: Filter stability test?

    // Test for infinite DC gain.
    // Pure integrators should not be implemented
    // using an ARMA filter.
    if (fabs(densum) > tol)
    {
        dcGain = numsum / densum;
    } else {
        // non-finite DC gain
        errorCode = 4;
    }

    return dcGain;
}

float SISOFilter::step(float in)
{

    this->_in = in;
    this->_out = den(0) * in;

    // NOTE: implicit state->a.k[0] == 1 because
    // we're assigning state->y.k[0] without a coefficient

    // update the output
    for (int k = 1; k < order() + 1; k++)
    {
        // TRICKY: because we haven't rolled the buffers yet
        // the input and output buffer indices are one less
        // than the coefficient indices
        this->_out += num(k) * uBuff(k - 1);
        this->_out -= den(k) * yBuff(k - 1);
    }

    // // roll the buffers.  Always keep the whole buffer in case we change filter coefficients.
    // for (int k = ARMA_SISO_MAX_ORDER; k > 0; k--)
    // {
    //     state->y[k] = state->y[k - 1];
    //     state->u[k] = state->u[k - 1];
    // }
    // state->y[0] = *out;
    // state->u[0] = in;

    roll_buffer(yBuff, this->out());
    roll_buffer(uBuff, this->in());

    return 0;
}

// resize vector to specified length by either truncating from the left or
// zero padding from the left
FiltVectorXf left_resize(const FiltVectorXf &in, char len)
{
    FiltVectorXf out(len);

    if (in.size() < len) {

        out << FiltVectorXf::Zero(len - in.size()), in;

        // // zero padding
        // out.head(len - in.size()).setZero();

        // // copy input
        // out.tail(in.size()) = in;
    } else {
        out << in.tail(len);
    }

    return out;
}

// resize vector to specified length by either truncating from the right or
// zero padding from the right
FiltVectorXf right_resize(const FiltVectorXf &in, char len)
{
    FiltVectorXf out(len);

    if (in.size() < len) {
 
        out << in, FiltVectorXf::Zero(len - in.size());

        // // zero padding
        // out.tail(len - in.size()).setZero();

        // // copy input
        // out.head(in.size()) = in;
    } else {
        out << in.head(len);
    }

    return out;
}

// resize vector to specified length by either truncating from the right or
// zero padding from the right
void roll_buffer(FiltVectorXf &buff, float u)
{

    if (buff.size() > 0) {
        buff << u, buff.head(buff.size() - 1);
    }

    // // zero padding
    // out.tail(len - in.size()).setZero();

    // // copy input
    // out.head(in.size()) = in;
}
