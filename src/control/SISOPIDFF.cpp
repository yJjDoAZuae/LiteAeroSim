#include "control/SISOPIDFF.hpp"

using namespace Control;

float SISOPIDFF::step(float cmdIn, float measIn, float measDotIn)
{
    // optional cmd/meas unwrapping
    if (unwrapInputs) {
        measUnwrap.step(measIn);
        cmdUnwrap.step(cmdIn, measUnwrap.out());
    } else {
        measUnwrap.reset(measIn);
        cmdUnwrap.reset(cmdIn);
    }

    cmdSignal.step(cmdUnwrap.out());
    measSignal.step(measUnwrap.out());
    errSignal.step(cmdSignal.out() - measSignal.out());
    ffwdSignal.step(cmdUnwrap.out());

    I.step(Ki*errSignal.out());

    measDotSignal.step(measDotIn);

    outSignal.step( feedfwd() + prop() + integ() + deriv() );

    return out();
}

float SISOPIDFF::step(float cmdIn, float measIn)
{
    return step(cmdIn, measIn, D.step(measIn));
}

void SISOPIDFF::reset(float cmdIn, float measIn, float measDotIn, float outIn)
{
    // optional cmd/meas unwrapping
    if (unwrapInputs) {
        measUnwrap.step(measIn);
        cmdUnwrap.step(cmdIn, measUnwrap.out());
    } else {
        measUnwrap.reset(measIn);
        cmdUnwrap.reset(cmdIn);
    }

    cmdSignal.resetInput(cmdUnwrap.out());
    measSignal.resetInput(measUnwrap.out());
    errSignal.resetInput(cmdSignal.out() - measSignal.out());
    ffwdSignal.resetInput(cmdUnwrap.out());

    D.reset(measIn, measDotIn);

    measDotSignal.resetInput(D.out());

    outSignal.resetOutput( outIn );

    I.reset(outIn - (feedfwd() + prop() + deriv()));

}

void SISOPIDFF::reset(float cmdIn, float measIn, float outIn)
{
    reset(cmdIn, measIn, outIn, 0.0f);
}
