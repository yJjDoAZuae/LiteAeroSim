

#pragma once

#include "control/SISOBlock.hpp"

namespace Control {

class Limit : public SISOBlock {

    public:

        Limit() : 
            _lowerLimit(0), 
            _upperLimit(0), 
            _limitedLower(false),
            _limitedUpper(false), 
            _enableLowerLimit(false), 
            _enableUpperLimit(false) 
            {}

        void disable() { disableLower(); disableUpper(); }
        void enable() { enableLower(); enableUpper(); }
        void disableLower() { _enableLowerLimit = false; step(_in); }
        void disableUpper() { _enableUpperLimit = false; step(_in); }
        void enableLower() { _enableLowerLimit = true; step(_in); }
        void enableUpper() { _enableUpperLimit = true; step(_in); }
        void setLower(float lim);
        void setUpper(float lim);
        void set(float lowerLim, float upperLim) { setLower(lowerLim); setUpper(upperLim); }

        float lowerLimit() const { return _lowerLimit; };
        float upperLimit() const { return _upperLimit; };
        bool isLimitedLower() const { return _limitedLower; };
        bool isLimitedUpper() const { return _limitedUpper; };
        bool isLimited() const { return isLimitedLower() || isLimitedUpper(); };
        bool isLowerEnabled() const { return _enableLowerLimit; }
        bool isUpperEnabled() const { return _enableUpperLimit; }

        float step(float u);

    protected:

        float _lowerLimit;
        float _upperLimit;
        bool _limitedLower;
        bool _limitedUpper;
        bool _enableLowerLimit;
        bool _enableUpperLimit;

};

}
