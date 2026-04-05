// Tests for KeyboardInput — roadmap item 1.
// Design authority: docs/architecture/manual_input.md

#include "input/KeyboardInput.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cstring>

using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// MockKeyState: a fake SDL key-state array.
// SDL_NUM_SCANCODES is defined in SDL_scancode.h; use a fixed size that covers
// all scancodes used in tests.
// ---------------------------------------------------------------------------

static constexpr int KEY_STATE_SIZE = 512;

struct MockKeyState {
    std::array<Uint8, KEY_STATE_SIZE> keys{};

    void press(SDL_Scancode code) {
        if (static_cast<int>(code) < KEY_STATE_SIZE) keys[code] = 1;
    }
    void release(SDL_Scancode code) {
        if (static_cast<int>(code) < KEY_STATE_SIZE) keys[code] = 0;
    }
    void releaseAll() { keys.fill(0); }

    KeyboardInput::KeyStateProvider provider() {
        return [this]() -> const Uint8* { return keys.data(); };
    }
};

// ---------------------------------------------------------------------------
// Helper: build a KeyboardInput with default config and a mock key provider.
// ---------------------------------------------------------------------------

static KeyboardInput makeKeyboard(MockKeyState& mock)
{
    KeyboardInput kb(mock.provider());
    kb.initialize(nlohmann::json::object());  // all defaults
    kb.reset();
    return kb;
}

// ---------------------------------------------------------------------------
// Test 1: No keys pressed → neutral command
// ---------------------------------------------------------------------------

TEST(KeyboardInputTest, NoKeysPressed_NeutralCommand)
{
    MockKeyState mock;
    auto kb = makeKeyboard(mock);

    const auto frame = kb.read(0.02f);

    EXPECT_FLOAT_EQ(frame.command.n_z,               1.0f);   // neutral_nz_g
    EXPECT_FLOAT_EQ(frame.command.n_y,               0.0f);
    EXPECT_FLOAT_EQ(frame.command.rollRate_Wind_rps,  0.0f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd,        0.05f);  // idle_throttle_nd
    EXPECT_EQ(frame.actions, 0u);
}

// ---------------------------------------------------------------------------
// Test 2: Pitch-up key held N steps → n_z increases correctly
// ---------------------------------------------------------------------------

TEST(KeyboardInputTest, PitchUpHeld_NzIncreases)
{
    MockKeyState mock;
    auto kb = makeKeyboard(mock);
    mock.press(SDL_SCANCODE_UP);

    const float dt_s   = 0.02f;
    const int   steps  = 5;
    for (int i = 0; i < steps; ++i) {
        kb.read(dt_s);
    }
    const auto frame = kb.read(dt_s);  // 6th step

    // nz_rate_g_s = 2.0, so after 6 steps: n_z = 1.0 + 2.0 * 6 * 0.02 = 1.24
    EXPECT_NEAR(frame.command.n_z, 1.0f + 2.0f * 6 * dt_s, 1e-4f);
}

// ---------------------------------------------------------------------------
// Test 3: n_z clamps at max_nz_g
// ---------------------------------------------------------------------------

TEST(KeyboardInputTest, NzClampsAtMax)
{
    MockKeyState mock;
    auto kb = makeKeyboard(mock);
    mock.press(SDL_SCANCODE_UP);

    // Run enough steps to exceed max_nz_g = 4.0
    for (int i = 0; i < 200; ++i) kb.read(0.02f);

    const auto frame = kb.read(0.02f);
    EXPECT_FLOAT_EQ(frame.command.n_z, 4.0f);
}

// ---------------------------------------------------------------------------
// Test 4: n_z clamps at min_nz_g
// ---------------------------------------------------------------------------

TEST(KeyboardInputTest, NzClampsAtMin)
{
    MockKeyState mock;
    auto kb = makeKeyboard(mock);
    mock.press(SDL_SCANCODE_DOWN);

    for (int i = 0; i < 200; ++i) kb.read(0.02f);

    const auto frame = kb.read(0.02f);
    EXPECT_FLOAT_EQ(frame.command.n_z, -2.0f);
}

// ---------------------------------------------------------------------------
// Test 5: Throttle ramps up and clamps at 1.0
// ---------------------------------------------------------------------------

TEST(KeyboardInputTest, ThrottleRampsUpAndClamps)
{
    MockKeyState mock;
    auto kb = makeKeyboard(mock);
    mock.press(SDL_SCANCODE_W);  // key_throttle_up

    // throttle_rate_nd_s = 0.5, dt = 0.1 → 0.05/step; 20 steps reach 1.0
    for (int i = 0; i < 100; ++i) kb.read(0.1f);

    const auto frame = kb.read(0.1f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd, 1.0f);
}

// ---------------------------------------------------------------------------
// Test 6: Throttle ramps down and clamps at 0.0
// ---------------------------------------------------------------------------

TEST(KeyboardInputTest, ThrottleRampsDownAndClamps)
{
    MockKeyState mock;
    auto kb = makeKeyboard(mock);
    mock.press(SDL_SCANCODE_S);  // key_throttle_down

    for (int i = 0; i < 100; ++i) kb.read(0.1f);

    const auto frame = kb.read(0.1f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd, 0.0f);
}

// ---------------------------------------------------------------------------
// Test 7: Center key → instant neutral and fires Center action (edge-triggered)
// ---------------------------------------------------------------------------

TEST(KeyboardInputTest, CenterKey_SnapsToNeutralAndFiresAction)
{
    MockKeyState mock;
    auto kb = makeKeyboard(mock);

    // Move n_z away from neutral first
    mock.press(SDL_SCANCODE_UP);
    for (int i = 0; i < 10; ++i) kb.read(0.02f);
    mock.releaseAll();

    // Press center
    mock.press(SDL_SCANCODE_SPACE);
    const auto frame = kb.read(0.02f);

    EXPECT_FLOAT_EQ(frame.command.n_z,               1.0f);
    EXPECT_FLOAT_EQ(frame.command.n_y,               0.0f);
    EXPECT_FLOAT_EQ(frame.command.rollRate_Wind_rps,  0.0f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd,        0.05f);
    EXPECT_TRUE(hasAction(frame, InputAction::Center));

    // Second tick with center still held → action bit must NOT fire again (edge-triggered)
    const auto frame2 = kb.read(0.02f);
    EXPECT_FALSE(hasAction(frame2, InputAction::Center));
}

// ---------------------------------------------------------------------------
// Test 8: reset() returns neutral command
// ---------------------------------------------------------------------------

TEST(KeyboardInputTest, Reset_ReturnsNeutralCommand)
{
    MockKeyState mock;
    auto kb = makeKeyboard(mock);
    mock.press(SDL_SCANCODE_UP);
    for (int i = 0; i < 10; ++i) kb.read(0.02f);
    mock.releaseAll();

    kb.reset();
    const auto frame = kb.read(0.02f);

    EXPECT_FLOAT_EQ(frame.command.n_z,               1.0f);
    EXPECT_FLOAT_EQ(frame.command.n_y,               0.0f);
    EXPECT_FLOAT_EQ(frame.command.rollRate_Wind_rps,  0.0f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd,        0.05f);
}

// ---------------------------------------------------------------------------
// Test 9: initialize() applies non-default config
// ---------------------------------------------------------------------------

TEST(KeyboardInputTest, Initialize_AppliesNonDefaultConfig)
{
    MockKeyState mock;
    KeyboardInput kb(mock.provider());

    nlohmann::json cfg;
    cfg["nz_rate_g_s"]         = 5.0;
    cfg["idle_throttle_nd"]    = 0.1;
    cfg["max_nz_g"]            = 6.0;
    cfg["neutral_nz_g"]        = 1.0;
    kb.initialize(cfg);
    kb.reset();

    // No keys: throttle should be idle_throttle_nd = 0.1
    const auto frame = kb.read(0.02f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd, 0.1f);

    // Pitch up for 1 step at 0.02s: n_z += 5.0 * 0.02 = 0.10 → 1.10
    mock.press(SDL_SCANCODE_UP);
    const auto frame2 = kb.read(0.02f);
    EXPECT_NEAR(frame2.command.n_z, 1.10f, 1e-4f);
}

// ---------------------------------------------------------------------------
// Test 10: initialize() with unrecognized fields is accepted (all have defaults)
// ---------------------------------------------------------------------------

TEST(KeyboardInputTest, Initialize_EmptyConfig_UsesDefaults)
{
    MockKeyState mock;
    KeyboardInput kb(mock.provider());
    // Empty config must not throw — all fields have defaults.
    EXPECT_NO_THROW(kb.initialize(nlohmann::json::object()));
}
