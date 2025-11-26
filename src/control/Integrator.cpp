#include "control/Integrator.hpp"

using namespace Control;

float Integrator::step(float in)
{

    bool awActive = false;

    for (int k = 0; k < aw.size(); k++) {
        awActive |= aw.at(k).isActive();
    }

    if (!awActive) {
        _out = limit.step(_out + 0.5*(in + _in) * _dt);
    } else {
        _out = limit.step(_out);
    }

    _in = in;
    return _out;
}
