#define _USE_MATH_DEFINES
#include "control/Unwrap.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <cmath>

using namespace liteaerosim::control;

TEST(UnwrapTest, Instantiation00) {

    Unwrap U;

    EXPECT_EQ(U.in(), 0.0f);
    EXPECT_EQ(U.out(), 0.0f);

}

TEST(UnwrapTest, JsonRoundTrip) {
    Unwrap U;
    U.initialize({{"schema_version", 1}});
    U.step(0.1f);
    U.step(0.2f);
    // Simulate wrap: step past pi, should unwrap to > pi
    U.step(static_cast<float>(M_PI) - 0.1f);
    U.step(-static_cast<float>(M_PI) + 0.1f);  // wraps; unwrapped output should be > pi

    nlohmann::json snap = U.serializeJson();
    Unwrap U2;
    U2.initialize({{"schema_version", 1}});
    U2.deserializeJson(snap);

    EXPECT_FLOAT_EQ(U2.step(0.5f), U.step(0.5f));
}

TEST(UnwrapTest, SchemaVersionMismatch) {
    Unwrap U;
    U.initialize({{"schema_version", 1}});
    nlohmann::json snap = U.serializeJson();
    snap["schema_version"] = 99;
    EXPECT_THROW(U.deserializeJson(snap), std::runtime_error);
}

TEST(UnwrapTest, SetReference_ThenStepGivesSameAsDirectRef) {
    const float ref = 2.0f;
    const float u   = -3.0f;  // far from ref, so wrap fires

    Unwrap U1, U2;
    float y1 = U1.step(u, ref);   // two-arg: unwrap u relative to ref

    U2.setReference(ref);          // new API: store reference
    float y2 = U2.step(u);         // single-arg NVI should use the stored ref

    EXPECT_FLOAT_EQ(y1, y2);
}

TEST(UnwrapTest, JsonRoundTrip_PreservesRef) {
    // After a two-arg step, the reference (= output) must survive serialization
    // so that the next single-arg step produces the correct continuation.
    Unwrap U;
    U.step(3.0f);       // ref_ → 3.0, out_ → 3.0
    nlohmann::json snap = U.serializeJson();

    Unwrap U2;
    U2.deserializeJson(snap);

    // step(-2.9) with ref_=3.0  => 3.0 + wrapToPi(-5.9) ≈ 3.38
    // step(-2.9) with ref_=0.0  => 0.0 + wrapToPi(-2.9) = -2.9
    // They differ, so if ref_ is lost the test fails.
    EXPECT_FLOAT_EQ(U2.step(-2.9f), U.step(-2.9f));
}
