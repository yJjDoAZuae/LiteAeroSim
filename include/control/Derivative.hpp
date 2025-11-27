#pragma once

#include "control/control.hpp"
#include "control/SISOBlock.hpp"
#include "control/Limit.hpp"
#include "control/Antiwindup.hpp"
#include <vector>

namespace Control {

class Derivative : public SISOBlock {

    public:

        Derivative() : _dt(1.0f), _Tau(0.0f), _method(DiscretizationMethod::FwdEuler) {}

        Limit limit;

        void reset(float u, float uDot) {
            _out = limit.step(uDot);
            _in = u;
        }

        float step(float u);
        void setDt(float dt) { _dt = (dt>1e-6) ? dt : 1.0f; };
        void setTau(float Tau) { _Tau = Tau; };
        float dt() const {return _dt;}
        float Tau() const {return _Tau;}
        void setMethod(DiscretizationMethod method) {_method = method;}

    protected:

        float _dt;
        float _Tau;
        DiscretizationMethod _method;

};

}
