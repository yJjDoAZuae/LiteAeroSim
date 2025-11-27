#include "control/SISOPIDFF.hpp"

using namespace Control;

float SISOPIDFF::step(float cmdIn, float measIn, float measDotIn)
{
    // TODO: optional cmd/meas unwrapping

    cmdSignal.step(cmdIn);
    measSignal.step(measIn);
    errSignal.step(cmdSignal.out() - measSignal.out());
    ffwdSignal.step(cmdIn);

    D.reset(measIn, measDotIn);
    I.step(Ki*errSignal.out());

    measDotSignal.step(D.out());

    outSignal.step( feedfwd() + prop() + integ() + deriv() );

    return out();
}

float SISOPIDFF::step(float cmdIn, float measIn)
{
    // TODO: optional cmd/meas unwrapping

    cmdSignal.step(cmdIn);
    measSignal.step(measIn);
    errSignal.step(cmdSignal.out() - measSignal.out());
    ffwdSignal.step(cmdIn);

    D.step(measIn);
    I.step(Ki*errSignal.out());

    measDotSignal.step(D.out());

    outSignal.step( feedfwd() + prop() + integ() + deriv() );

    return out();
}

void SISOPIDFF::reset(float cmdIn, float measIn, float measDotIn, float outIn)
{
    // TODO: optional cmd/meas unwrapping

    cmdSignal.resetInput(cmdIn);
    measSignal.resetInput(measIn);
    errSignal.resetInput(cmdSignal.out() - measSignal.out());
    ffwdSignal.resetInput(cmdIn);

    D.reset(measIn, measDotIn);

    measDotSignal.resetInput(D.out());

    outSignal.resetOutput( outIn );

    I.reset(outIn - (feedfwd() + prop() + deriv()));

}

void SISOPIDFF::reset(float cmdIn, float measIn, float outIn)
{
    // TODO: optional cmd/meas unwrapping

    cmdSignal.resetInput(cmdIn);
    measSignal.resetInput(measIn);
    errSignal.resetInput(cmdSignal.out() - measSignal.out());
    ffwdSignal.resetInput(cmdIn);

    D.reset(measIn, 0.0f);

    measDotSignal.resetInput(D.out());

    outSignal.resetOutput( outIn );

    I.reset(outIn - (feedfwd() + prop() + deriv()));
}
