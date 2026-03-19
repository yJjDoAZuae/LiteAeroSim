#include "control/FilterFIR.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using namespace liteaerosim::control;
using namespace liteaerosim;

TEST(FilterFIRTest, Instantiation00) {
    FilterFIR G;
    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);
    EXPECT_EQ(G.order(), 0);
}

TEST(FilterFIRTest, AverageFilter3Step) {
    FilterFIR G;
    G.setAverageFIR(2);
    EXPECT_EQ(G.order(), 2);
    EXPECT_FLOAT_EQ(G.dcGain(), 1.0f);
    // First step: only one sample so output = 1/3
    EXPECT_FLOAT_EQ(G.step(1.0f), 1.0f / 3.0f);
}

TEST(FilterFIRTest, SchemaVersionAndTypeName) {
    FilterFIR G;
    nlohmann::json snap = G.serializeJson();
    EXPECT_EQ(snap["schema_version"].get<int>(), 1);
    EXPECT_EQ(snap["type"].get<std::string>(), "FilterFIR");
}

TEST(FilterFIRTest, NviInAndOutUpdated) {
    FilterFIR G;
    G.setAverageFIR(2);
    SisoElement* base = &G;
    base->step(1.0f);
    EXPECT_EQ(base->in(), 1.0f);
    EXPECT_NE(base->out(), 0.0f);
}

TEST(FilterFIRTest, SerializeNonEmpty) {
    FilterFIR G;
    G.setAverageFIR(2);
    G.step(1.0f);
    G.step(1.0f);
    nlohmann::json snap = G.serializeJson();
    EXPECT_EQ(snap["schema_version"].get<int>(), 1);
    EXPECT_EQ(snap["type"].get<std::string>(), "FilterFIR");
    EXPECT_TRUE(snap.contains("order"));
    EXPECT_TRUE(snap.contains("state"));
}

TEST(FilterFIRTest, JsonRoundTrip) {
    FilterFIR G;
    G.setAverageFIR(4);
    G.step(1.0f);
    G.step(0.5f);
    G.step(0.2f);
    nlohmann::json snap = G.serializeJson();
    FilterFIR G2;
    G2.setAverageFIR(4);
    G2.deserializeJson(snap);
    EXPECT_FLOAT_EQ(G2.step(1.0f), G.step(1.0f));
}
