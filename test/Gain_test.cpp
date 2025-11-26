#define _USE_MATH_DEFINES
#include "control/Gain.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

using namespace Control;

TEST(GainTest, Instantiation00) {

    Gain<float, 3> K;

    EXPECT_EQ(K.in(), 0.0f);
    EXPECT_EQ(K.out(), 0.0f);
}

TEST(GainTest, Read00) {

    Gain<float, 3> K;
    std::ifstream fs;

    fs.open("tmp.json");

    EXPECT_EQ(K.in(), 0.0f);
    EXPECT_EQ(K.out(), 0.0f);

    std::stringstream ss;
    if (fs) {
        ss << fs.rdbuf();
        fs.close();
        // tab.readJSON(ss);
        EXPECT_EQ(K.readJSON(ss), 0);
    }

}

