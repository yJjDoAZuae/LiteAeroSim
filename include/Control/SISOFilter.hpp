
#include "Control/SISOBlock.hpp"
#include <Eigen/Dense>

#define FILTER_MAX_STATES 31

// A single input, single output discrete filter implementation
// with ARMA parameterization
// NOTE: filter parameterization enforces finite DC gain
template <char NUM_STATES=FILTER_MAX_STATES> 
class SISOFilter : SISOBlock {

    public:

        SISOFilter() : num(0),
               den(0),
               uBuff(0),
               yBuff(0),
               n(0) {};

        SISOFilter(SISOFilter &filt) 
        {
            copy(filt)
        };

        void copy(SISOFilter &filt);

        // IIR filter design
        void setLowPassFirstIIR(float dt, float tau); // first order low pass filter design
        void setLowPassSecondIIR(float dt, float wn_rps, float zeta); // second order low pass filter design
        void setHighPassFirstIIR(float dt, float tau); // first order high pass filter design
        void setHighPassSecondIIR(float dt, float wn_rps, float zeta); // second order high pass filter design
        void setDerivIIR(float dt, float tau);  // first order derivative + low pass filter design
        void setNotchSecondIIR(float dt, float wn_rps, float zeta);  // second order notch filter design

        void setDim(char nDim);

        // FIR filter design
        void setButterworthFIR(char dim); // Butterworth low pass FIR filter design
        void setAverageFIR(char dim); // equal weight moving average FIR filter design
        void setExpFIR(char dim, float tau); // exponential decaying weight moving average FIR filter design

        // step the filter
        float step(float in);  

        // reset the fiter based on inputs
        void reset_input(float in);

        // Reset the filter based on outputs
        // If dc gain is zero, then the filter is
        // reset to zero regardless of argument value
        void reset_output(float out);

        // dc gain value of the filter
        float dc_gain();

    private:

        char n; // filter dimension

        Eigen::Matrix<float, Eigen::Dynamic, 1, 0, NUM_STATES+1, 1> num;
        Eigen::Matrix<float, Eigen::Dynamic, 1, 0, NUM_STATES+1, 1> den;

        Eigen::Matrix<float, Eigen::Dynamic, 1, 0, NUM_STATES+1, 1> uBuff;
        Eigen::Matrix<float, Eigen::Dynamic, 1, 0, NUM_STATES+1, 1> yBuff;

        // Tustin discrete IIR filter realization
        void tustin_first();
        void tustin_second();

};
