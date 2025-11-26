#define _USE_MATH_DEFINES
#include "control/Gain.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

using namespace Control;

TEST(GainTest, Instantiation00) {

    TableAxis<float> axis;
    RectilinearTable<float, float, 3> tab;
    Gain K;
    std::ifstream fs;

    fs.open("tmp.json");

    json data = json::parse("{\"hello\": \"world\"}");

    EXPECT_EQ(data.is_boolean(), false);

    EXPECT_EQ(K.in(), 0.0f);
    EXPECT_EQ(K.out(), 0.0f);

    EXPECT_EQ(axis.name.size(), 0);
    EXPECT_EQ(axis.domain.size(), 0);

    EXPECT_EQ(axis.readJSON(std::stringstream("{\"hello\": \"world\"}")), 0);

    EXPECT_EQ(tab.numRecords(), 0);
    EXPECT_EQ(tab.read("foo.json"), 0);

    std::stringstream ss;
    if (fs) {
        ss << fs.rdbuf();
        fs.close();
        // tab.readJSON(ss);
        EXPECT_EQ(axis.readJSON(ss), 0);
    }

    EXPECT_EQ(axis.domain.size(), 0);

}
