// Tests for JoystickInput — roadmap item 1.
// Design authority: docs/architecture/manual_input.md
//
// All tests use the AxisProvider test constructor: no SDL initialization or
// physical device is required.

#include "input/JoystickInput.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <memory>

using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// Helper: build a JoystickInput with a fixed axis provider and default config.
// JoystickInput is non-movable, so returned via unique_ptr.
// ---------------------------------------------------------------------------

static std::unique_ptr<JoystickInput> makeJoystick(
    std::function<Sint16(int)> provider,
    const nlohmann::json& config = nlohmann::json::object())
{
    auto js = std::make_unique<JoystickInput>(std::move(provider));
    js->initialize(config);
    js->reset();
    return js;
}

// Constant provider: all axes return the same raw value.
static auto constProvider(Sint16 raw)
{
    return [raw](int) -> Sint16 { return raw; };
}

// Per-axis provider: map from axis index → raw value.
static auto axisProvider(std::initializer_list<std::pair<const int,Sint16>> mappings)
{
    std::map<int,Sint16> m(mappings);
    return [m](int axis) -> Sint16 {
        auto it = m.find(axis);
        return it != m.end() ? it->second : Sint16(0);
    };
}

// ---------------------------------------------------------------------------
// Test 1: Axis at center → output = center_output
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, AxisAtCenter_OutputIsCenterOutput)
{
    // raw = 0 → normalized ≈ 0 → after dead zone → 0 → output = center_output
    auto js = makeJoystick(constProvider(0));
    const auto frame = js->read();

    EXPECT_NEAR(frame.command.n_z,               1.0f,   1e-3f);  // center_output = 1.0
    EXPECT_NEAR(frame.command.n_y,               0.0f,   1e-3f);
    EXPECT_NEAR(frame.command.rollRate_Wind_rps,  0.0f,   1e-3f);
    EXPECT_NEAR(frame.command.throttle_nd,        0.5f,   1e-3f);  // center_output = 0.5
}

// ---------------------------------------------------------------------------
// Test 2: Axis at dead zone boundary → output = center_output
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, AxisAtDeadZoneBoundary_OutputIsCenterOutput)
{
    // dead_zone = 0.05; raw such that |r| = 0.05 exactly.
    // For default limits raw_min=-32768, raw_max=32767:
    // half_range = (32767-(-32768))/2 = 32767.5
    // r = raw / 32767.5 = 0.05 → raw = 0.05 * 32767.5 ≈ 1638
    const Sint16 boundary_raw = static_cast<Sint16>(std::round(0.05f * 32767.5f));

    // nz_axis: inverted, so we use negative boundary to get |r|=0.05
    nlohmann::json cfg;
    cfg["dead_zone_nd"] = 0.05;

    auto js = makeJoystick([boundary_raw](int axis) -> Sint16 {
        // nz_axis index=1, ny_axis index=3, roll_axis index=0, throttle_axis index=2
        return boundary_raw;
    }, cfg);

    const auto frame = js->read();
    // All axes at the dead zone boundary → all outputs = center_output
    EXPECT_NEAR(frame.command.n_z,               1.0f, 1e-2f);
    EXPECT_NEAR(frame.command.n_y,               0.0f, 1e-2f);
    EXPECT_NEAR(frame.command.rollRate_Wind_rps,  0.0f, 1e-2f);
}

// ---------------------------------------------------------------------------
// Test 3: Axis just beyond dead zone → output non-zero (continuity)
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, AxisJustBeyondDeadZone_NonZeroOutput)
{
    // raw slightly beyond dead zone boundary
    const Sint16 beyond_raw = static_cast<Sint16>(std::round(0.06f * 32767.5f));

    auto js = makeJoystick([beyond_raw](int axis) -> Sint16 {
        if (axis == 0) return beyond_raw;  // roll_axis (not inverted)
        return Sint16(0);
    });

    const auto frame = js->read();
    // roll_axis should have non-zero output
    EXPECT_GT(std::abs(frame.command.rollRate_Wind_rps), 0.0f);
}

// ---------------------------------------------------------------------------
// Test 4: Axis at full positive deflection → output = center_output + scale
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, AxisAtFullPositiveDeflection_MaxOutput)
{
    // ny_axis: not inverted, center=0, scale=1.0; full +raw → normalized +1
    // → dead zone applied → r' = (1 - 0.05)/(1 - 0.05) = 1.0 → output = 0 + 1.0 = 1.0
    auto js = makeJoystick([](int axis) -> Sint16 {
        if (axis == 3) return Sint16(32767);  // ny_axis full forward
        return Sint16(0);
    });

    const auto frame = js->read();
    EXPECT_NEAR(frame.command.n_y, 1.0f, 1e-2f);
}

// ---------------------------------------------------------------------------
// Test 5: Axis at full negative deflection → correct output (SDL asymmetric min)
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, AxisAtFullNegativeDeflection_ClampedCorrectly)
{
    // SDL min is -32768, which normalizes to just beyond -1.0; must be clamped.
    auto js = makeJoystick([](int axis) -> Sint16 {
        if (axis == 3) return Sint16(-32768);  // ny_axis full back
        return Sint16(0);
    });

    const auto frame = js->read();
    // Must not produce output beyond -(center_output + scale) = -1.0
    EXPECT_GE(frame.command.n_y, -1.0f);
}

// ---------------------------------------------------------------------------
// Test 6: Inverted axis — positive raw → negative output
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, InvertedAxis_PositiveRawNegativeOutput)
{
    // nz_axis: inverted=true, center=1.0, scale=3.0
    // Full positive raw (+32767) → normalized +1 → inverted → -1
    // → output = 1.0 + (-1.0) * 3.0 = -2.0 (clamped to min_nz_g = -2.0)
    auto js = makeJoystick([](int axis) -> Sint16 {
        if (axis == 1) return Sint16(32767);  // nz_axis full positive
        return Sint16(0);
    });

    const auto frame = js->read();
    EXPECT_NEAR(frame.command.n_z, -2.0f, 1e-2f);
}

// ---------------------------------------------------------------------------
// Test 7: n_z clamped at max_nz_g
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, NzClampedAtMax)
{
    // nz_axis inverted: full negative raw → normalized -1 → inverted +1
    // → output = 1.0 + 3.0 = 4.0 → clamped at max_nz_g = 4.0
    auto js = makeJoystick([](int axis) -> Sint16 {
        if (axis == 1) return Sint16(-32768);
        return Sint16(0);
    });

    const auto frame = js->read();
    EXPECT_NEAR(frame.command.n_z, 4.0f, 1e-2f);
}

// ---------------------------------------------------------------------------
// Test 8: n_z = 1.0 g at stick center (axis = 0)
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, NzAtStickCenter_Is1g)
{
    auto js = makeJoystick(constProvider(0));
    const auto frame = js->read();
    EXPECT_NEAR(frame.command.n_z, 1.0f, 1e-3f);
}

// ---------------------------------------------------------------------------
// Test 9: reset() returns neutral command
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, Reset_ReturnsNeutralCommand)
{
    auto js = makeJoystick(constProvider(0));

    // Modify config's idle_throttle_nd to verify reset uses it
    nlohmann::json cfg;
    cfg["idle_throttle_nd"] = 0.1;
    js->initialize(cfg);
    js->reset();

    const auto frame = js->read();
    EXPECT_NEAR(frame.command.n_z,               1.0f, 1e-3f);
    EXPECT_NEAR(frame.command.n_y,               0.0f, 1e-3f);
    EXPECT_NEAR(frame.command.rollRate_Wind_rps,  0.0f, 1e-3f);
}

// ---------------------------------------------------------------------------
// Test 10: initialize() applies non-default dead zone
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, Initialize_AppliesNonDefaultDeadZone)
{
    nlohmann::json cfg;
    cfg["dead_zone_nd"] = 0.20;

    // raw slightly below the new dead zone (20% of 32767.5 ≈ 6554)
    const Sint16 inside_raw = static_cast<Sint16>(std::round(0.19f * 32767.5f));

    auto js = makeJoystick([inside_raw](int axis) -> Sint16 {
        if (axis == 0) return inside_raw;  // roll_axis
        return Sint16(0);
    }, cfg);

    const auto frame = js->read();
    EXPECT_NEAR(frame.command.rollRate_Wind_rps, 0.0f, 1e-3f);
}

// ---------------------------------------------------------------------------
// Test 11: initialize() rejects dead_zone_nd >= 1.0
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, Initialize_InvalidDeadZone_Throws)
{
    JoystickInput js(constProvider(0));

    nlohmann::json cfg;
    cfg["dead_zone_nd"] = 1.0;
    EXPECT_THROW(js.initialize(cfg), std::invalid_argument);

    cfg["dead_zone_nd"] = 1.5;
    EXPECT_THROW(js.initialize(cfg), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Test 12: Calibrated raw_min/raw_max sub-range: raw at raw_max → normalized +1
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, CalibratedSubRange_RawMaxNormalizesToPlusOne)
{
    // Configure ny_axis with sub-range [-16000, 16000]
    nlohmann::json cfg;
    cfg["ny_axis"] = {
        {"sdl_axis_index", 3}, {"center_output", 0.0}, {"scale", 1.0},
        {"inverted", false}, {"raw_min", -16000}, {"raw_max", 16000}, {"raw_trim", 0}
    };
    cfg["dead_zone_nd"] = 0.0;  // disable dead zone for precise check

    auto js = makeJoystick([](int axis) -> Sint16 {
        if (axis == 3) return Sint16(16000);  // at raw_max
        return Sint16(0);
    }, cfg);

    const auto frame = js->read();
    // normalized → +1, output = 0 + 1.0 = 1.0, clamped at max_ny_g = 2.0
    EXPECT_NEAR(frame.command.n_y, 1.0f, 1e-2f);
}

// ---------------------------------------------------------------------------
// Test 13: Out-of-range raw clamped to [raw_min, raw_max]
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, OutOfRangeRaw_Clamped)
{
    // Configure ny_axis with sub-range [-16000, 16000]; inject raw=32767 (beyond max)
    nlohmann::json cfg;
    cfg["ny_axis"] = {
        {"sdl_axis_index", 3}, {"center_output", 0.0}, {"scale", 1.0},
        {"inverted", false}, {"raw_min", -16000}, {"raw_max", 16000}, {"raw_trim", 0}
    };
    cfg["dead_zone_nd"] = 0.0;

    auto js = makeJoystick([](int axis) -> Sint16 {
        if (axis == 3) return Sint16(32767);  // beyond raw_max=16000
        return Sint16(0);
    }, cfg);

    const auto frame = js->read();
    // Clamped to raw_max → normalized +1 → output = 1.0 (same as test 12)
    EXPECT_NEAR(frame.command.n_y, 1.0f, 1e-2f);
}

// ---------------------------------------------------------------------------
// Test 14: isConnected() is true for injected-provider instance
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, IsConnected_TrueForInjectedProvider)
{
    auto js = makeJoystick(constProvider(0));
    EXPECT_TRUE(js->isConnected());
}

// ---------------------------------------------------------------------------
// Test 15: After simulated disconnect, read() returns neutral command
// ---------------------------------------------------------------------------

TEST(JoystickInputTest, SimulatedDisconnect_ReturnsNeutralCommand)
{
    // Set throttle sub-range so non-neutral axis values would produce non-neutral output.
    nlohmann::json cfg;
    cfg["idle_throttle_nd"] = 0.05;

    auto js = makeJoystick([](int axis) -> Sint16 {
        return Sint16(32767);  // all axes at full deflection
    }, cfg);

    js->simulateDisconnect();
    EXPECT_FALSE(js->isConnected());

    const auto frame = js->read();
    // After disconnect, neutral command must be returned regardless of axis provider.
    EXPECT_NEAR(frame.command.n_z,               1.0f,  1e-3f);
    EXPECT_NEAR(frame.command.n_y,               0.0f,  1e-3f);
    EXPECT_NEAR(frame.command.rollRate_Wind_rps,  0.0f,  1e-3f);
    EXPECT_NEAR(frame.command.throttle_nd,        0.05f, 1e-3f);
}
