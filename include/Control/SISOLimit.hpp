

#pragma once

#include "control/SISOBlock.hpp"

namespace Control {

class SISOLimit : public SISOBlock {

    public:

        SISOLimit() : 
            _lowerLimit(0), 
            _upperLimit(0), 
            _limitedLower(false),
            _limitedUpper(false), 
            _enableLowerLimit(false), 
            _enableUpperLimit(false) 
            {}

        // SISOLimit(const SISOLimit &lim) { copy(lim); }
        // void copy(const SISOLimit &lim);

        void disable() { _enableLowerLimit=false; _enableUpperLimit = false; }
        void enable() { _enableLowerLimit=true; _enableUpperLimit = true; }
        void disableLower() { _enableLowerLimit = false; }
        void disableUpper() { _enableUpperLimit = false; }
        void enableLower() { _enableLowerLimit = true; }
        void enableUpper() { _enableUpperLimit = true; }
        void setLower(float lim);
        void setUpper(float lim);
        void set(float lowerLim, float upperLim) { setLower(lowerLim); setUpper(upperLim); }

        float lowerLimit() const { return _lowerLimit; };
        float upperLimit() const { return _upperLimit; };
        bool isLimitedLower() const { return _limitedLower; };
        bool isLimitedUpper() const { return _limitedUpper; };
        bool lowerEnabled() const { return _enableLowerLimit; }
        bool upperEnabled() const { return _enableUpperLimit; }

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
