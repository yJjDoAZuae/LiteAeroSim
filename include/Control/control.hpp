#pragma once

#include <Eigen/Dense>

namespace Control {

#define FILTER_MAX_STATES 31
#define NUM_STATES FILTER_MAX_STATES

// template <char NUM_STATES=FILTER_MAX_STATES>
typedef Eigen::Matrix<float, Eigen::Dynamic, 1, 0, NUM_STATES + 1, 1> FiltVectorXf;

typedef Eigen::Matrix<float, 2, 2> Mat22;
typedef Eigen::Matrix<float, 2, 1> Mat21;
typedef Eigen::Matrix<float, 1, 2> Mat12;
typedef Eigen::Matrix<float, 1, 1> Mat11;

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, 0, NUM_STATES, NUM_STATES> MatNN;
typedef Eigen::Matrix<float, Eigen::Dynamic,              1, 0, NUM_STATES,          1> MatN1;
typedef Eigen::Matrix<float,              1, Eigen::Dynamic, Eigen::RowMajor,          1, NUM_STATES> Mat1N;
typedef Eigen::Matrix<float,              1,              1, 0,          1,          1> Mat11;

FiltVectorXf left_resize(const FiltVectorXf &in, char len);
FiltVectorXf right_resize(const FiltVectorXf &in, char len);
void roll_buffer(FiltVectorXf &buff, float u);


typedef enum {

    // error_code values
    NONE = 0, // no error
    INVALID_DIMENSION = 1, // invalid vector or matrix dimension input
    INVALID_TIMESTEP = 2, // invalid timestep
    UNSTABLE = 4, // invalid denominator coefficients
    INFINITE_DC_GAIN = 8, // non-finite dc gain
    ZERO_DC_GAIN = 16, // attempt to perform output initialization with zero dc gain
    INVALID_POLYNOMIAL = 32

} FilterError;

}
