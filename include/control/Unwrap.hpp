

#pragma once

#include "control/SISOBlock.hpp"

namespace Control {

class Unwrap : public SISOBlock {

    public:

        Unwrap() {}

        // unwrap relative to previous output value
        float step(float u);

        // unwrap relative to an externally provided reference
        float step(float u, float ref);

        void reset(float u) { _in = u; _out = u; };

};

}
