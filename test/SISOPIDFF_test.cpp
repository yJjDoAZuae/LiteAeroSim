#include "control/SISOPIDFF.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <stdexcept>

using namespace liteaerosim::control;

TEST(SISOPIDFFTest, SchemaVersionAndTypeName) {
    SISOPIDFF pid;
    nlohmann::json snap = pid.serializeJson();
    EXPECT_EQ(snap["schema_version"].get<int>(), 1);
    EXPECT_EQ(snap["type"].get<std::string>(), "SISOPIDFF");
}

TEST(SISOPIDFFTest, StepReturnsOutputValue) {
    SISOPIDFF pid;
    float result = pid.step(1.0f, 0.0f);
    EXPECT_EQ(pid.output(), result);
}

TEST(SISOPIDFFTest, JsonRoundTrip) {
    SISOPIDFF pid;
    pid.initialize({{"schema_version", 1}});
    pid.step(1.0f, 0.5f);
    pid.step(2.0f, 1.0f);

    nlohmann::json snap = pid.serializeJson();
    SISOPIDFF pid2;
    pid2.initialize({{"schema_version", 1}});
    pid2.deserializeJson(snap);

    EXPECT_FLOAT_EQ(pid2.step(1.5f, 1.0f), pid.step(1.5f, 1.0f));
}

TEST(SISOPIDFFTest, SchemaVersionMismatch_Throws) {
    SISOPIDFF pid;
    nlohmann::json snap = pid.serializeJson();
    snap["schema_version"] = 99;
    EXPECT_THROW(pid.deserializeJson(snap), std::runtime_error);
}
