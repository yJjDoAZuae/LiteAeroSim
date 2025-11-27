#include "control/SISOPIDFF.hpp"

using namespace Control;

float SISOPIDFF::step(float cmdIn, float measIn, float measDotIn)
{

    cmdSignal.step(cmdIn);
    measSignal.step(measIn);
    errSignal.step(cmdSignal.out() - measSignal.out());
    ffwdSignal.step(cmdIn);

    D.reset(measSignal.out(), measDotIn);
    I.step(Ki*errSignal.out());

    outSignal.step( feedfwd() + prop() + integ() + deriv() );

    return out();
}
