

#pragma once

#include "control/Limit.hpp"

namespace Control {

class RateLimit : public Limit {

    public:

        RateLimit() : _dt(1.0f) {}
        RateLimit(const RateLimit& lim) : _dt(lim._dt) {}

        void disable() { disableLower(); disableUpper(); }  // make sure we call the RateLimit version
        void enable() { enableLower(); enableUpper(); } // make sure we call the RateLimit version
        void disableLower() { _enableLowerLimit = false; }
        void disableUpper() { _enableUpperLimit = false; }
        void enableLower() { _enableLowerLimit = true; }
        void enableUpper() { _enableUpperLimit = true; }
        void setLower(float lim);
        void setUpper(float lim);
        void setDt(float dt) { _dt = (dt>1e-6) ? dt : 1.0f; };

        void reset(float u);
        float step(float u);

        float dt() const {return _dt;}

    protected:

        float _dt;

};

}
