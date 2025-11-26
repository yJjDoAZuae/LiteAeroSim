#define _USE_MATH_DEFINES
#include "control/Unwrap.hpp"
#include <gtest/gtest.h>

using namespace Control;

TEST(UnwrapTest, Instantiation00) {

    Unwrap U;

    EXPECT_EQ(U.in(), 0.0f);
    EXPECT_EQ(U.out(), 0.0f);

}
