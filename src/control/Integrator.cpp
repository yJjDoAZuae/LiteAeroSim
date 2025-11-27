#include "control/Integrator.hpp"

using namespace Control;

float Integrator::step(float in)
{

    bool awActive = false;

    for (int k = 0; k < aw.size(); k++) {
        awActive |= aw.at(k).isActive();
    }

    // TODO: Backsolving
    // For a backsolving implementation we only want to change the
    // integrator state based on SISO saturation of the 
    // integrator's output that occurs immediately downstream of the
    // output of this class.  We do not want to feed back any
    // filter dynamics or other static mappings.  Saturation information that
    // cannot be returned exactly risks causing integrator windup --
    // the very phenomenon that backsolving portends to mitigate.
    // If the saturation cannot be returned exactly, then it is safer
    // (and still reasonably performant) to use an antiwindup 
    // detection mode and freeze the integrator rather than attempting 
    // to correct it through backsolving.
    // TODO: investigate a method that applies a small signal deadzone
    // to suppress inexact backsolving while still performing backsolving
    // on large errors.
    // TODO: investigate dynamic inversion to approximately recover
    // the unfiltered version of a signal for use in backsolving
    // calculations.

    if (!awActive) {

        switch (_method) {

        case DiscretizationMethod::BackEuler:
            // Backward Euler
            _out = limit.step(_out + in * _dt);
            break;

        case DiscretizationMethod::FwdEuler:
            // Forward Euler
            _out = limit.step(_out + _in * _dt);
            break;

        case DiscretizationMethod::Bilinear:
            // Bilinear (Tustin)
            _out = limit.step(_out + 0.5*(in + _in) * _dt);
            break;
        }
    } else {
        _out = limit.step(_out);
    }

    _in = in;
    return _out;
}
