#define _USE_MATH_DEFINES
#include "Control/FilterSS2.hpp"
#include <gtest/gtest.h>

using namespace Control;

TEST(FilterSS2Test, Instantiation) {

    FilterSS2 G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

}

