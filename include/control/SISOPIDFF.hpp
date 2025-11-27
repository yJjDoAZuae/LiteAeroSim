#pragma once

#include "control/SISOBlock.hpp"
#include "control/control.hpp"
#include "control/Gain.hpp"
#include "control/FilterSS2Clip.hpp"
#include "control/Integrator.hpp"
#include "control/Derivative.hpp"

namespace Control {

class SISOPIDFF
{

public:
    FilterSS2Clip cmd;
    FilterSS2Clip ffwd;
    FilterSS2Clip meas;
    FilterSS2Clip err;
    FilterSS2Clip out;

    Gain Kp;
    Gain Ki;
    Gain Kd;
    Gain Kff;

    Integrator I;
    Derivative D;

    SISOPIDFF() : _useInternalMeasDot(false) {}

    SISOPIDFF(const SISOPIDFF &pid);
    {
        copy(pid);
    }

    void copy(const SISOPIDFF &pid);

    // step the PIDFF with internally calculated measurement derivative
    float step(float cmdIn, float measIn);

    // step the PIDFF with externally provided measurement derivative
    float step(float cmdIn, float measIn, float measDotIn);

    // reset the PIDFF based on cmd, meas, and output
    void reset(float cmdIn, float measIn, float outIn);

    // reset the PIDFF based on cmd, meas, measDot, and output
    void reset(float cmdIn, float measIn, float measDotIn, float outIn);

    float cmd() const { return cmd.in(); }

    float meas() const { return meas.in(); }

    float out() const { return out.out(); }

    float err() const { return err.in(); }

    // return the feedforward term
    float feedfwd() const { return Kff * ffwd.out(); }

    // return the proportional term
    float prop() const { return Kp * err.out(); }

    // return the derivative term
    float deriv() const { return Kd * D.out(); }

    // return the integrator state (gain is upstream of the integrator)
    float integ() const {return I.out();}

protected:

    bool _useInternalMeasDot; // TODO: is this needed?

};

}

