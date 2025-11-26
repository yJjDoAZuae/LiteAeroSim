#pragma once

#include "control/SISOBlock.hpp"
#include "control/RectilinearTable.hpp"

#include <vector>
#include <string>
#include <sstream>

namespace Control {

template <typename T, uint32_t NumAxes>    
class Gain : public SISOBlock {

    public:

        Gain() : _K(0) {}

        float step(float u) { _in = u; _out = K()*u; return out(); };
        void schedule(std::array<T, NumAxes> u);
        T K() const {return _K;}
        void reset(float u) { _in = u; _out = u; };
        int readJSON(std::stringstream& ss);
        int readFile(std::string filepath);

    protected:

        RectilinearTable<T, T, NumAxes> table;
        T _K;

};


template <typename T, uint32_t NumAxes>
int Gain<T,NumAxes>::readJSON(std::stringstream& ss)
{
    json data;
    data = json::parse(ss);

    return 0;

}

template <typename T, uint32_t NumAxes>
int Gain<T,NumAxes>::readFile(std::string filepath)
{
    std::ifstream fs;

    fs.open(filepath);

    std::stringstream ss;

    json data;
    if (fs) {
        ss << fs.rdbuf();
        fs.close();
        // tab.readJSON(ss);
        readJSON(ss);
    }

    return 0;

}

}
