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

    bool useInternalMeasDot;

    SISOPIDFF() : useInternalMeasDot(false) {}

    SISOPIDFF(const SISOPIDFF &pid)
    {
        copy(pid);
    }

    void copy(const SISOPIDFF &pid);

    // step the PIDFF
    float step(float cmdIn, float measIn, float measDotIn);

    // reset the PIDFF based on cmd, meas, and output
    void reset(float cmdIn, float measIn, float outIn);

    float cmd() const
    {
        return cmd.in();
    }

    float meas() const
    {
        return meas.in();
    }

    float out() const
    {
        return out.out();
    }

    float err() const
    {
        return err.in();
    }

    //
    float feedfwd() const
    {
        return ffwd.out();
    }

    // return the proportional term
    float prop() const
    {
        return Kp * err.out();
    }

    //
    float deriv() const {}

    //
    float integ() const {}
};

}
