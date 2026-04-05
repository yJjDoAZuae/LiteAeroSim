// Tests for ScriptedInput — roadmap item 1.
// Design authority: docs/architecture/manual_input.md

#include "input/ScriptedInput.hpp"
#include <gtest/gtest.h>
#include <thread>
#include <atomic>

using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// Test 1: push() → read() returns the pushed command
// ---------------------------------------------------------------------------

TEST(ScriptedInputTest, PushThenRead_ReturnsPushedCommand)
{
    ScriptedInput si;
    si.initialize(nlohmann::json::object());
    si.reset();

    AircraftCommand cmd;
    cmd.n_z               = 2.5f;
    cmd.n_y               = 0.3f;
    cmd.rollRate_Wind_rps = 0.7f;
    cmd.throttle_nd       = 0.8f;
    si.push(cmd);

    const auto frame = si.read();
    EXPECT_FLOAT_EQ(frame.command.n_z,               2.5f);
    EXPECT_FLOAT_EQ(frame.command.n_y,               0.3f);
    EXPECT_FLOAT_EQ(frame.command.rollRate_Wind_rps, 0.7f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd,       0.8f);
    EXPECT_EQ(frame.actions, 0u);
}

// ---------------------------------------------------------------------------
// Test 2: reset() → read() returns neutral command
// ---------------------------------------------------------------------------

TEST(ScriptedInputTest, Reset_ReturnsNeutralCommand)
{
    ScriptedInput si;
    si.initialize(nlohmann::json::object());

    // Push a non-neutral command first
    AircraftCommand cmd;
    cmd.n_z               = 3.0f;
    cmd.n_y               = 1.0f;
    cmd.rollRate_Wind_rps = 1.5f;
    cmd.throttle_nd       = 0.9f;
    si.push(cmd);

    si.reset();
    const auto frame = si.read();

    // After reset, neutral: n_z=1, n_y=0, rollRate=0, throttle=0
    EXPECT_FLOAT_EQ(frame.command.n_z,               1.0f);
    EXPECT_FLOAT_EQ(frame.command.n_y,               0.0f);
    EXPECT_FLOAT_EQ(frame.command.rollRate_Wind_rps, 0.0f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd,       0.0f);
    EXPECT_EQ(frame.actions, 0u);
}

// ---------------------------------------------------------------------------
// Test 3: initialize() accepts empty JSON config without throwing
// ---------------------------------------------------------------------------

TEST(ScriptedInputTest, Initialize_EmptyConfig_DoesNotThrow)
{
    ScriptedInput si;
    EXPECT_NO_THROW(si.initialize(nlohmann::json::object()));
}

// ---------------------------------------------------------------------------
// Test 4: push() from one thread, read() from another — no data race
// (Run under ThreadSanitizer to verify; logic check verifies correctness.)
// ---------------------------------------------------------------------------

TEST(ScriptedInputTest, ConcurrentPushRead_NoDataRace)
{
    ScriptedInput si;
    si.initialize(nlohmann::json::object());
    si.reset();

    constexpr int kIterations = 10000;
    std::atomic<int> errors{0};

    // Writer thread: push incrementing n_z values
    std::thread writer([&]() {
        for (int i = 0; i < kIterations; ++i) {
            AircraftCommand cmd;
            cmd.n_z         = static_cast<float>(i);
            cmd.n_y         = 0.0f;
            cmd.rollRate_Wind_rps = 0.0f;
            cmd.throttle_nd = 0.0f;
            si.push(cmd);
        }
    });

    // Reader thread: read and verify value is non-negative (always valid)
    std::thread reader([&]() {
        for (int i = 0; i < kIterations; ++i) {
            const auto frame = si.read();
            if (frame.command.n_z < 0.0f) {
                ++errors;
            }
        }
    });

    writer.join();
    reader.join();

    EXPECT_EQ(errors.load(), 0);
}
