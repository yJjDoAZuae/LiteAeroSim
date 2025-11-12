
#include "Control/SISOBlock.hpp"
#include <Eigen/Dense>

#define FILTER_MAX_STATES 31
#define NUM_STATES FILTER_MAX_STATES


namespace Control {

// template <char NUM_STATES=FILTER_MAX_STATES>
typedef Eigen::Matrix<float, Eigen::Dynamic, 1, 0, NUM_STATES + 1, 1> FiltVectorXf;

// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
// template <char NUM_STATES=FILTER_MAX_STATES>
class SISOFilter : SISOBlock
{

public:
    SISOFilter() : errorCode(0)
    {
        num << 1;
        den << 1;
        uBuff << 0;
        yBuff << 0;
    }

    SISOFilter(SISOFilter &filt)
    {
        copy(filt);
    }

    void copy(SISOFilter &filt);

    // IIR filter design
    void setLowPassFirstIIR(float dt, float tau);                  // first order low pass filter design
    void setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero);  // second order low pass filter design
    void setHighPassFirstIIR(float dt, float tau);                 // first order high pass filter design
    void setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero); // second order high pass filter design
    void setDerivIIR(float dt, float tau);                         // first order derivative + low pass filter design
    void setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num);    // second order notch filter design

    char order() { return den.rows() - 1; }

    // FIR filter design
    void setButterworthFIR(char order);    // Butterworth low pass FIR filter design
    void setAverageFIR(char order);        // equal weight moving average FIR filter design
    void setExpFIR(char order, float tau); // exponential decaying weight moving average FIR filter design

    // step the filter
    float step(float in);

    // reset the fiter based on inputs
    void resetInput(float in);

    // Reset the filter based on outputs
    // If dc gain is zero, then the filter is
    // reset to zero regardless of argument value
    void resetOutput(float out);

    // dc gain value of the filter
    float dcGain();

    // retrieve the last errorCode generated
    int lastError() { return errorCode; };

private:

    int errorCode=0;

    FiltVectorXf num;
    FiltVectorXf den;

    FiltVectorXf uBuff;
    FiltVectorXf yBuff;

    // Tustin discrete IIR filter realization
    void tustin_first();
    void tustin_second();
};

}