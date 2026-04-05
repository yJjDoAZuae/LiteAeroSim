#include "input/KeyboardInput.hpp"
#include <algorithm>
#include <stdexcept>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------

KeyboardInput::KeyboardInput(KeyStateProvider provider)
    : key_provider_(std::move(provider))
{}

// ---------------------------------------------------------------------------

KeyboardInput::KeyStateProvider KeyboardInput::defaultKeyStateProvider()
{
    return []() -> const Uint8* {
        return SDL_GetKeyboardState(nullptr);
    };
}

// ---------------------------------------------------------------------------

void KeyboardInput::initialize(const nlohmann::json& config)
{
    // All fields are optional; defaults from KeyboardInputConfig are used.
    auto get = [&](const char* key, auto& field) {
        if (config.contains(key)) {
            using T = std::remove_reference_t<decltype(field)>;
            field = static_cast<T>(config.at(key).get<double>());
        }
    };

    get("nz_rate_g_s",           config_.nz_rate_g_s);
    get("ny_rate_g_s",           config_.ny_rate_g_s);
    get("roll_rate_rate_rad_s2", config_.roll_rate_rate_rad_s2);
    get("throttle_rate_nd_s",    config_.throttle_rate_nd_s);
    get("min_nz_g",              config_.min_nz_g);
    get("max_nz_g",              config_.max_nz_g);
    get("max_ny_g",              config_.max_ny_g);
    get("max_roll_rate_rad_s",   config_.max_roll_rate_rad_s);
    get("idle_throttle_nd",      config_.idle_throttle_nd);
    get("neutral_nz_g",          config_.neutral_nz_g);

    // Integer scancode fields
    auto getKey = [&](const char* key, SDL_Scancode& field) {
        if (config.contains(key)) {
            field = static_cast<SDL_Scancode>(config.at(key).get<int>());
        }
    };
    getKey("key_pitch_up",      config_.key_pitch_up);
    getKey("key_pitch_down",    config_.key_pitch_down);
    getKey("key_roll_right",    config_.key_roll_right);
    getKey("key_roll_left",     config_.key_roll_left);
    getKey("key_yaw_right",     config_.key_yaw_right);
    getKey("key_yaw_left",      config_.key_yaw_left);
    getKey("key_throttle_up",   config_.key_throttle_up);
    getKey("key_throttle_down", config_.key_throttle_down);
    getKey("key_center",        config_.key_center);
    getKey("key_sim_reset",     config_.key_sim_reset);
    getKey("key_sim_start",     config_.key_sim_start);

    reset();
}

// ---------------------------------------------------------------------------

void KeyboardInput::reset()
{
    command_         = neutralCommand();
    prev_center_     = false;
    prev_sim_reset_  = false;
    prev_sim_start_  = false;
}

// ---------------------------------------------------------------------------

AircraftCommand KeyboardInput::neutralCommand() const
{
    AircraftCommand cmd;
    cmd.n_z               = config_.neutral_nz_g;
    cmd.n_y               = 0.0f;
    cmd.rollRate_Wind_rps = 0.0f;
    cmd.throttle_nd       = config_.idle_throttle_nd;
    return cmd;
}

// ---------------------------------------------------------------------------

ManualInputFrame KeyboardInput::read()
{
    return read(dt_s_);
}

// ---------------------------------------------------------------------------

ManualInputFrame KeyboardInput::read(float dt_s)
{
    dt_s_ = dt_s;
    const Uint8* keys = key_provider_();

    uint32_t actions = 0u;
    applyKeys(keys, dt_s, actions);

    ManualInputFrame frame;
    frame.command = command_;
    frame.actions = actions;
    return frame;
}

// ---------------------------------------------------------------------------

void KeyboardInput::setKeyStateProvider(KeyStateProvider provider)
{
    key_provider_ = std::move(provider);
}

// ---------------------------------------------------------------------------

static bool keyDown(const Uint8* keys, SDL_Scancode code)
{
    if (code == SDL_SCANCODE_UNKNOWN) return false;
    return keys[static_cast<int>(code)] != 0;
}

void KeyboardInput::applyKeys(const Uint8* keys, float dt_s, uint32_t& actions_out)
{
    actions_out = 0u;

    // --- Center key: edge-triggered action + immediate snap ---
    const bool center_now = keyDown(keys, config_.key_center);
    if (center_now && !prev_center_) {
        command_    = neutralCommand();
        actions_out |= static_cast<uint32_t>(InputAction::Center);
    }
    prev_center_ = center_now;

    // If center was just pressed, skip axis integration this tick.
    if (hasAction({command_, actions_out}, InputAction::Center)) {
        // Still process action keys below.
        goto action_keys;
    }

    // --- Axis integration ---
    command_.n_z               += (keyDown(keys, config_.key_pitch_up)   ?  config_.nz_rate_g_s           : 0.0f) * dt_s;
    command_.n_z               -= (keyDown(keys, config_.key_pitch_down)  ?  config_.nz_rate_g_s           : 0.0f) * dt_s;
    command_.n_y               += (keyDown(keys, config_.key_yaw_right)   ?  config_.ny_rate_g_s           : 0.0f) * dt_s;
    command_.n_y               -= (keyDown(keys, config_.key_yaw_left)    ?  config_.ny_rate_g_s           : 0.0f) * dt_s;
    command_.rollRate_Wind_rps += (keyDown(keys, config_.key_roll_right)  ?  config_.roll_rate_rate_rad_s2 : 0.0f) * dt_s;
    command_.rollRate_Wind_rps -= (keyDown(keys, config_.key_roll_left)   ?  config_.roll_rate_rate_rad_s2 : 0.0f) * dt_s;
    command_.throttle_nd       += (keyDown(keys, config_.key_throttle_up) ?  config_.throttle_rate_nd_s   : 0.0f) * dt_s;
    command_.throttle_nd       -= (keyDown(keys, config_.key_throttle_down)? config_.throttle_rate_nd_s   : 0.0f) * dt_s;

    clampCommand();

    action_keys:
    // --- Named action keys: edge-triggered ---
    {
        const bool sim_reset_now = keyDown(keys, config_.key_sim_reset);
        if (sim_reset_now && !prev_sim_reset_) {
            actions_out |= static_cast<uint32_t>(InputAction::SimReset);
        }
        prev_sim_reset_ = sim_reset_now;

        const bool sim_start_now = keyDown(keys, config_.key_sim_start);
        if (sim_start_now && !prev_sim_start_) {
            actions_out |= static_cast<uint32_t>(InputAction::SimStart);
        }
        prev_sim_start_ = sim_start_now;
    }
}

// ---------------------------------------------------------------------------

void KeyboardInput::clampCommand()
{
    command_.n_z               = std::clamp(command_.n_z,
                                            config_.min_nz_g, config_.max_nz_g);
    command_.n_y               = std::clamp(command_.n_y,
                                            -config_.max_ny_g, config_.max_ny_g);
    command_.rollRate_Wind_rps = std::clamp(command_.rollRate_Wind_rps,
                                            -config_.max_roll_rate_rad_s,
                                             config_.max_roll_rate_rad_s);
    command_.throttle_nd       = std::clamp(command_.throttle_nd, 0.0f, 1.0f);
}

}  // namespace liteaero::simulation
