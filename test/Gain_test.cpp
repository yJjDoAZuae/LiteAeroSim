#define _USE_MATH_DEFINES
#include "control/Gain.hpp"
#include <gtest/gtest.h>

using namespace liteaerosim::control;

TEST(GainTest, Instantiation00) {
    Gain<float, 3> K;
    EXPECT_EQ(K.value(), 0.0f);
}

TEST(GainTest, Set00) {
    Gain<float, 3> K;

    EXPECT_EQ(K.value(), 0.0f);
    K.set(5);
    EXPECT_EQ(K.value(), 5.0f);

    K.set(-5);
    EXPECT_EQ(K.value(), -5.0f);
}

TEST(GainTest, Multiply00) {
    Gain<float, 3> K;

    K.set(5);
    EXPECT_EQ(K.value(), 5.0f);

    EXPECT_EQ(K * 10.0f, 50.0f);
    EXPECT_EQ(10.0f * K, 50.0f);
}

TEST(GainTest, Divide00) {
    Gain<float, 3> K;

    K.set(5);
    EXPECT_EQ(K.value(), 5.0f);

    EXPECT_FLOAT_EQ(K / 10.0f, 0.5f);
    EXPECT_FLOAT_EQ(10.0f / K, 2.0f);
}

TEST(GainTest, Add00) {
    Gain<float, 3> K;

    K.set(5);
    EXPECT_EQ(K.value(), 5.0f);

    EXPECT_EQ(K + 10.0f, 15.0f);
    EXPECT_EQ(10.0f + K, 15.0f);
}

TEST(GainTest, Subtract00) {
    Gain<float, 3> K;

    K.set(5);
    EXPECT_EQ(K.value(), 5.0f);

    EXPECT_EQ(K - 10.0f, -5.0f);
    EXPECT_EQ(10.0f - K, 5.0f);
}
