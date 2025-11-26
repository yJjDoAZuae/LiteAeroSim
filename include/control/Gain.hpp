#pragma once

#include "control/SISOBlock.hpp"
#include "control/RectilinearTable.hpp"

#include <vector>
#include <string>

namespace Control {

class Gain : public SISOBlock {

    public:

        Gain() : _K(0) {}

        float step(float u) { _in = u; _out = K()*u; return out(); };
        void schedule(std::array<float, 3> u);
        float K() const {return _K;}
        void reset(float u) { _in = u; _out = u; };
        void readTable(std::string filePath);

    protected:

        RectilinearTable<float, float, 3> table;
        float _K;

};

}
