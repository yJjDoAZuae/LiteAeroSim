#pragma once

#include "control/SISOBlock.hpp"
#include "control/Limit.hpp"
#include "control/Antiwindup.hpp"
#include <vector>

namespace Control {
typedef enum {
    FwdEuler = 0,
    BackEuler = 1,
    Bilinear = 2
} IntegMethod;

class Integrator : public SISOBlock {

    public:

        Integrator() : _dt(1.0f), _method(IntegMethod::FwdEuler) {}

        Limit limit;
        std::vector<Antiwindup> aw;

        void reset(float u);

        float step(float u);
        void setDt(float dt) { _dt = (dt>1e-6) ? dt : 1.0f; };
        float dt() const {return _dt;}
        void setMethod(IntegMethod method) {_method = method;}

    protected:

        float _dt;
        IntegMethod _method;

};

}
