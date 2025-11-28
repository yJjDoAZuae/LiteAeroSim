
#include "control/SISOPIDFF.hpp"

namespace Control {

class ControlLoop {

public:

    SISOPIDFF pid;

    virtual void configure() = 0;
    virtual void configure() = 0;
    virtual float step(float cmdIn, float measIn) = 0;
    virtual void reset(float cmdIn, float measIn) = 0;

}

}
