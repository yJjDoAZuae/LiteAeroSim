#pragma once

#include "control/SISOBlock.hpp"

#include <string>
#include <sstream>
#include <vector>

namespace Control {

template <typename Ta>
class TableAxis {

    public:

        std::string name;
        std::vector<Ta> domain;

        int read(std::string filePath);

        int readJSON(std::stringstream& ss);

};

template <typename Ta>
int TableAxis<Ta>::readJSON(std::stringstream& ss) 
{
    json data = json::parse(ss);

    return 0;
}

template <typename Tv, typename Ta, uint32_t NumAxes>
class RectilinearTable {

    public:

        RectilinearTable() {}

        uint32_t numRecords() const { return value.size(); }

        // data records indexing is in axis order
        // i.e. value[k] is at axis1[k1], axis2[k2], axis3[k3]
        // where
        // k1 = k - axis2.size()*k2 - axis2.size()*axis3.size()*k3
        // k2 = k%axis2.size() - axis3.size()*k3
        // k3 = k%(axis2.size()*axis3.size())

        Tv lookup(std::array<Ta, NumAxes> u); // index of lower corner of the grid cell that contains lookup

        int read(std::string filePath);

    protected:

        std::array<uint32_t, NumAxes> lookup_index(std::array<Ta, NumAxes> u); // index of lower corner of the grid cell that contains lookup
        std::array<Ta, NumAxes> lookup_frac(std::array<Ta, NumAxes> u); // scheduling fraction on each axis

        // grid point validity to allow non uniform grid boundaries
        std::vector<bool> valid;

        // grid point value
        std::vector<Tv> value;

        // lookup axis values
        TableAxis<Ta> axis[NumAxes];
        // TableAxis axis[NumAxes];

};

template <typename Tv, typename Ta, uint32_t NumAxes>
int RectilinearTable<Tv,Ta,NumAxes>::read(std::string filePath)
{
    std::ifstream fs;

    fs.open(filePath);

    std::stringstream ss;

    json data;
    if (fs) {
        ss << fs.rdbuf();
        fs.close();
        // tab.readJSON(ss);
        data = json::parse(ss);
    }

    return 0;

}

}
