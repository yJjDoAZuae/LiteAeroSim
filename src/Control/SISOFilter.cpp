#include "math.h"
// #include <stdio.h>
// #include <string.h>

#include "Control/SISOFilter.hpp"

static float dcTol = 1e-6;

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

// int tustin_1(arma_buffer_t *num, arma_buffer_t *den, float dt, arma_buffer_t *numz, arma_buffer_t *denz)
// {
//     const float tol = 1.0e-6;

//     float K = 2.0f / dt;
//     float coeff_denom = *den[0] * K + *den[1];

//     // printf("tustin_1: den->k[0] = %0.3f, den->k[1] = %0.3f, K = %0.3f\n", den->k[0], den->k[1], K);

//     if (fabs(coeff_denom) < tol)
//     {
//         return 1;
//     }

//     *numz[0] = (*num[0] * K + *num[1]) / coeff_denom;
//     *numz[1] = (-*num[0] * K + *num[1]) / coeff_denom;

//     *denz[0] = 1.0f;
//     *denz[1] = (-*den[0] * K + *den[1]) / coeff_denom;

//     return 0;
// }

// int tustin_2(arma_buffer_t *num, arma_buffer_t *den, float dt, arma_buffer_t *numz, arma_buffer_t *denz)
// {
//     const float tol = 1.0e-6;

//     float K = 2.0f / dt;
//     float coeff_denom = *den[0] * K * K + *den[1] * K + *den[2];

//     if (fabs(coeff_denom) < tol)
//     {
//         return 1;
//     }

//     *numz[0] = (*num[0] * K * K + *num[1] * K + *num[2]) / coeff_denom;
//     *numz[1] = (-2.0 * (*num)[0] * K * K + 2.0 * (*num)[2]) / coeff_denom;
//     *numz[2] = (*num[0] * K * K - *num[1] * K + *num[2]) / coeff_denom;

//     *denz[0] = 1.0f;
//     *denz[1] = (-2.0 * (*den)[0] * K * K + 2.0 * (*den)[2]) / coeff_denom;
//     *denz[2] = (*den[0] * K * K - *den[1] * K + *den[2]) / coeff_denom;

//     return 0;
// }

// int arma_siso_lowpass_1_design(float dt, float tau, arma_siso_filter_state_t *state)
// {

//     arma_buffer_t numz;
//     arma_buffer_t denz;

//     arma_buffer_init(&numz);
//     arma_buffer_init(&denz);

//     // HACK: zero order hold realization
//     // ydot = -1/tau y + 1/tau u
//     // yk+1 - yk = (-1/tau yk + 1/tau u) * dt
//     // yk+1 = (1 - dt/tau) xk + dt/tau u

//     // numz.k[0] = dt/tau;
//     // numz.k[1] = 0.0f;
//     // denz.k[0] = 1.0f;
//     // denz.k[1] = -(1-dt/tau);

//     // Tustin (bilinear) realization
//     arma_buffer_t num;
//     arma_buffer_t den;

//     arma_buffer_init(&num);
//     arma_buffer_init(&den);

//     num[0] = 0.0f;
//     num[1] = 1 / tau;
//     den[0] = 1.0f;
//     den[1] = 1 / tau;

//     // printf("num[0] = %0.3f, num[1] = %0.3f, den[0] = %0.3f, den[1] = %0.3f\n", num.k[0], num.k[1], den.k[0], den.k[1]);

//     int rc = tustin_1(&num, &den, dt, &numz, &denz);
//     if (rc != 0)
//     {
//         // printf("tustin returned %d\n", rc);
//         return rc;
//     }

//     // printf("numz[0] = %0.3f, numz[1] = %0.3f, denz[0] = %0.3f, denz[1] = %0.3f\n", numz.k[0], numz.k[1], denz.k[0], denz.k[1]);

//     return arma_siso_filter_coef_update(&numz, &denz, 1, state);
// }

// int arma_siso_lowpass_2_design(float dt, float wn, float zeta, float tau_zero, arma_siso_filter_state_t *state)
// {

//     arma_buffer_t numz;
//     arma_buffer_t denz;

//     arma_buffer_init(&numz);
//     arma_buffer_init(&denz);

//     // Tustin (bilinear) realization
//     arma_buffer_t num;
//     arma_buffer_t den;

//     arma_buffer_init(&num);
//     arma_buffer_init(&den);

//     num[0] = 0.0f;
//     num[1] = tau_zero;
//     num[2] = wn * wn;
//     den[0] = 1.0f;
//     den[1] = 2.0 * zeta * wn;
//     den[2] = wn * wn;

//     int rc = tustin_2(&num, &den, dt, &numz, &denz);
//     if (rc != 0)
//     {
//         return rc;
//     }

//     return arma_siso_filter_coef_update(&numz, &denz, 2, state);
// }

// int arma_siso_notch_2_design(float dt, float wn, float zeta_den,
//                              float zeta_num, arma_siso_filter_state_t *state)
// {

//     arma_buffer_t numz;
//     arma_buffer_t denz;

//     arma_buffer_init(&numz);
//     arma_buffer_init(&denz);

//     // Tustin (bilinear) realization
//     arma_buffer_t num;
//     arma_buffer_t den;

//     arma_buffer_init(&num);
//     arma_buffer_init(&den);

//     num[0] = 1.0f;
//     num[1] = 2.0 * zeta_num * wn;
//     num[2] = wn * wn;
//     den[0] = 1.0f;
//     den[1] = 2.0 * zeta_den * wn;
//     den[2] = wn * wn;

//     int rc = tustin_2(&num, &den, dt, &numz, &denz);
//     if (rc != 0)
//     {
//         return rc;
//     }

//     return arma_siso_filter_coef_update(&numz, &denz, 2, state);
// }

// int arma_siso_highpass_1_design(float dt, float tau, arma_siso_filter_state_t *state)
// {
//     arma_buffer_t numz;
//     arma_buffer_t denz;

//     arma_buffer_init(&numz);
//     arma_buffer_init(&denz);

//     // Tustin (bilinear) realization
//     arma_buffer_t num;
//     arma_buffer_t den;

//     arma_buffer_init(&num);
//     arma_buffer_init(&den);

//     num[0] = 1 / tau;
//     num[1] = 0.0f;
//     den[0] = 1.0f;
//     den[1] = 1 / tau;

//     int rc = tustin_1(&num, &den, dt, &numz, &denz);
//     if (rc != 0)
//     {
//         return rc;
//     }

//     return arma_siso_filter_coef_update(&numz, &denz, 1, state);
// }

// int arma_siso_highpass_2_design(float dt, float wn, float zeta, float c_zero, arma_siso_filter_state_t *state)
// {
//     arma_buffer_t numz;
//     arma_buffer_t denz;

//     arma_buffer_init(&numz);
//     arma_buffer_init(&denz);

//     // Tustin (bilinear) realization
//     arma_buffer_t num;
//     arma_buffer_t den;

//     arma_buffer_init(&num);
//     arma_buffer_init(&den);

//     num[0] = 1.0f;
//     num[1] = c_zero * 2.0 * zeta * wn;
//     num[2] = 0.0f;
//     den[0] = 1.0f;
//     den[1] = 2.0 * zeta * wn;
//     den[2] = wn * wn;

//     int rc = tustin_2(&num, &den, dt, &numz, &denz);
//     if (rc != 0)
//     {
//         return rc;
//     }

//     return arma_siso_filter_coef_update(&numz, &denz, 2, state);
// }

// int arma_siso_deriv_1_design(float dt, float tau, arma_siso_filter_state_t *state)
// {
//     arma_buffer_t numz;
//     arma_buffer_t denz;

//     arma_buffer_init(&numz);
//     arma_buffer_init(&denz);

//     // Tustin (bilinear) realization
//     arma_buffer_t num;
//     arma_buffer_t den;

//     arma_buffer_init(&num);
//     arma_buffer_init(&den);

//     num[0] = 1 / tau;
//     num[1] = 0.0f;
//     den[0] = 1.0f;
//     den[1] = 1 / tau;

//     int rc = tustin_1(&num, &den, dt, &numz, &denz);
//     if (rc != 0)
//     {
//         return rc;
//     }

//     return arma_siso_filter_coef_update(&numz, &denz, 1, state);
// }

// int arma_siso_filter_coef_update(arma_buffer_t *num,
//                                  arma_buffer_t *den, int n, arma_siso_filter_state_t *state)
// {

//     arma_buffer_init(&(state->a));
//     arma_buffer_init(&(state->b));

//     if (n < 0 || n > ARMA_SISO_MAX_ORDER)
//     {
//         return 1;
//     }

//     state->n = n;

//     for (int k = 0; k < state->n + 1; k++)
//     {
//         state->b[k] = *num[k];
//         state->a[k] = *den[k];
//     }

//     // we ignore the first entry in den and set it to 1 in all cases
//     state->a[0] = 1.0f;

//     return 0;
// }

// int arma_siso_filter_input_reset(float in, arma_siso_filter_state_t *state)
// {
//     const float tol = 1e-6;

//     arma_buffer_init(&(state->u));
//     arma_buffer_init(&(state->y));

//     float dcgain;
//     int rc = arma_siso_filter_dcgain(&dcgain, state);

//     if (rc == 0)
//     {
//         for (int k = 0; k < ARMA_SISO_MAX_ORDER; k++)
//         {
//             state->u[k] = in;
//             state->y[k] = in * dcgain;
//         }
//     }

//     return 0;
// }

// int arma_siso_filter_output_reset(float out, arma_siso_filter_state_t *state)
// {
//     const float tol = 1e-6;

//     arma_buffer_init(&(state->u));
//     arma_buffer_init(&(state->y));

//     float dcgain;
//     int rc = arma_siso_filter_dcgain(&dcgain, state);

//     if (rc == 0 && fabs(dcgain) > tol)
//     {
//         for (int k = 0; k < ARMA_SISO_MAX_ORDER; k++)
//         {
//             state->y[k] = out;
//             state->u[k] = out / dcgain;
//         }
//     }
//     else
//     {
//         return 1;
//     }

//     return 0;
// }

// int arma_siso_filter_dcgain(float *dcgain, arma_siso_filter_state_t *state)
// {
//     const float tol = 1e-6;

//     // TRICKY: potential access to uninitialized data
//     // for zeroth order (static gain) case.
//     float numsum = 0.0f;
//     float densum = 0.0f;

//     for (int k = 0; k < state->n + 1; k++)
//     {
//         numsum += state->b[k];
//         densum += state->a[k];
//     }

//     // We need an arbitrary finite DC gain to accomodate
//     // band pass, high pass, and lead/lag filters filters
//     // TODO: Filter stability test?
//     *dcgain = 0.0f;

//     // Test for infinite DC gain.
//     // Pure integrators should not be implemented
//     // using an ARMA filter.
//     if (fabs(densum) > tol)
//     {
//         *dcgain = numsum / densum;

//         // printf("dcgain = %0.3f\n", dcgain);
//     }
//     else
//     {
//         return 1;
//     }

//     return 0;
// }

// int arma_siso_filter_update(float in, float *out, arma_siso_filter_state_t *state)
// {

//     *out = state->b[0] * in;

//     // NOTE: implicit state->a.k[0] == 1 because
//     // we're assigning state->y.k[0] without a coefficient

//     // update the output
//     for (int k = 1; k < state->n + 1; k++)
//     {
//         // TRICKY: because we haven't rolled the buffers yet
//         // the input and output buffer indices are one less
//         // than the coefficient indices
//         *out += state->b[k] * state->u[k - 1];
//         *out -= state->a[k] * state->y[k - 1];
//     }

//     // roll the buffers.  Always keep the whole buffer in case we change filter coefficients.
//     for (int k = ARMA_SISO_MAX_ORDER; k > 0; k--)
//     {
//         state->y[k] = state->y[k - 1];
//         state->u[k] = state->u[k - 1];
//     }
//     state->y[0] = *out;
//     state->u[0] = in;

//     return 0;
// }

// // Init a FIR buffer to zeros
// int fir_buffer_init(fir_buffer_t *buff)
// {
//     memset(*buff, 0, FIR_SISO_MAX_ORDER * sizeof(fir_value_t));
//     return 0;
// }

// // Initialize a FIR filter
// int fir_siso_filter_init(fir_siso_filter_state_t *state)
// {

//     int rc = 0;

//     state->n = 0;

//     rc = fir_buffer_init(&state->b);
//     if (rc != 0)
//     {
//         return rc;
//     }

//     rc = fir_buffer_init(&state->u);
//     if (rc != 0)
//     {
//         return rc;
//     }

//     return rc;
// }

// // Reset a filter based on a constant input value
// int fir_siso_filter_dcgain(fir_value_t *dcgain, fir_siso_filter_state_t *state)
// {
//     long long tmp_out = 0;

//     if (state->n > FIR_SISO_MAX_ORDER)
//     {
//         return 1;
//     }

//     for (int k = 0; k < state->n; k++)
//     {
//         tmp_out += state->b[k];
//     }

//     *dcgain = tmp_out / state->n;

//     return 0;
// }

// // Reset a filter based on a constant input value
// int fir_siso_filter_input_reset(fir_value_t in, fir_siso_filter_state_t *state)
// {
//     if (state->n > FIR_SISO_MAX_ORDER)
//     {
//         return 1;
//     }

//     for (int k = state->n - 1; k >= 0; k--)
//     {
//         state->u[k] = in;
//     }

//     return 0;
// }

// // Reset a filter based on a constant output value
// int fir_siso_filter_output_reset(fir_value_t out, fir_siso_filter_state_t *state)
// {
//     long long sum_b = 0;

//     if (state->n > FIR_SISO_MAX_ORDER)
//     {
//         return 1;
//     }

//     for (int k = 0; k < state->n; k++)
//     {
//         sum_b += state->b[k];
//     }

//     fir_value_t in = 0;
//     if (sum_b != 0)
//     {
//         in = out * state->n / sum_b;
//     }
//     else
//     {
//         in = 0;
//     }

//     for (int k = 0; k < state->n; k++)
//     {
//         state->u[k] = in;
//     }

//     if (sum_b == 0)
//     {
//         return 2;
//     }

//     return 0;
// }

// // Step a filter forward one iteration
// int fir_siso_filter_update(fir_value_t in, fir_value_t *out, fir_siso_filter_state_t *state)
// {
//     long long tmp_out = 0;

//     if (state->n > FIR_SISO_MAX_ORDER)
//     {
//         return 1;
//     }

//     for (int k = state->n - 1; k > 0; k--)
//     {
//         state->u[k] = state->u[k - 1];
//         tmp_out += state->u[k] * state->b[k];
//     }
//     state->u[0] = in;
//     tmp_out += state->u[0] * state->b[0];

//     *out = tmp_out / state->n;

//     return 0;
// }
