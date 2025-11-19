#pragma once

class SISOBlock {

    public:

        float in() {
            return _in;
        }

        float out() {
            return _out;
        }

        SISOBlock() : _in(0), _out(0) {}

        // iteration interface
        virtual float step(float u)=0;

    protected:

        float _in;
        float _out;

};
