#pragma once

class SISOBlock {

    public:

        virtual ~SISOBlock() {};

        virtual float in() const = 0;
        virtual float out() const = 0;
        virtual operator float() const = 0;

        SISOBlock() {}

        // iteration interface
        virtual float step(float u)=0;

};
