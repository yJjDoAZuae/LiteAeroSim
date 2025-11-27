#pragma once

class SISOBlock {

    public:

        float in() const {
            return _in;
        }

        float out() const {
            return _out;
        }

        SISOBlock() : _in(0), _out(0) {}

        // iteration interface
        virtual float step(float u)=0;

    protected:

        float _in;
        float _out;

};
