#pragma once

#include "control/SISOBlock.hpp"
#include "control/Limit.hpp"
#include "control/Antiwindup.hpp"
#include <vector>

namespace Control {

class Integrator : public SISOBlock {

    public:

        Integrator() : _dt(1.0f) {}

        Limit limit;
        std::vector<Antiwindup> aw;

        void reset(float u);

        float step(float u);
        void setDt(float dt) { _dt = (dt>1e-6) ? dt : 1.0f; };
        float dt() const {return _dt;}

    protected:

        float _dt;


};

}
