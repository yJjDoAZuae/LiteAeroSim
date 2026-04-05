#pragma once

// Design authority: docs/architecture/manual_input.md

#include "input/ManualInput.hpp"
#include <SDL2/SDL.h>
#include <functional>

namespace liteaero::simulation {

struct KeyboardInputConfig {
    // SDL scancodes for each command axis.
    SDL_Scancode key_pitch_up        = SDL_SCANCODE_UP;
    SDL_Scancode key_pitch_down      = SDL_SCANCODE_DOWN;
    SDL_Scancode key_roll_right      = SDL_SCANCODE_RIGHT;
    SDL_Scancode key_roll_left       = SDL_SCANCODE_LEFT;
    SDL_Scancode key_yaw_right       = SDL_SCANCODE_E;
    SDL_Scancode key_yaw_left        = SDL_SCANCODE_Q;
    SDL_Scancode key_throttle_up     = SDL_SCANCODE_W;
    SDL_Scancode key_throttle_down   = SDL_SCANCODE_S;
    SDL_Scancode key_center          = SDL_SCANCODE_SPACE;

    // Keys that fire InputAction events (edge-triggered on key-down).
    // Set to SDL_SCANCODE_UNKNOWN to disable.
    SDL_Scancode key_sim_reset       = SDL_SCANCODE_UNKNOWN;
    SDL_Scancode key_sim_start       = SDL_SCANCODE_UNKNOWN;

    // Rates (per second) at which each command axis ramps while the key is held.
    float nz_rate_g_s            = 2.0f;
    float ny_rate_g_s            = 1.0f;
    float roll_rate_rate_rad_s2  = 1.0f;
    float throttle_rate_nd_s     = 0.5f;

    // Command limits — output is clamped to these ranges.
    float min_nz_g               = -2.0f;
    float max_nz_g               =  4.0f;
    float max_ny_g               =  2.0f;
    float max_roll_rate_rad_s    =  1.57f;
    float idle_throttle_nd       =  0.05f;

    // Neutral n_z (straight-and-level = 1 g).
    float neutral_nz_g           =  1.0f;
};

class KeyboardInput final : public ManualInput {
public:
    using KeyStateProvider = std::function<const Uint8*()>;

    // Constructs with the default production key-state provider
    // (SDL_GetKeyboardState).  Pass a custom provider for unit testing.
    explicit KeyboardInput(KeyStateProvider provider = defaultKeyStateProvider());

    void             initialize(const nlohmann::json& config) override;
    void             reset() override;

    // Read current keyboard state and advance the integrated command.
    // Uses the dt_s set by setTimestep() (default 0.02 s).
    ManualInputFrame read() override;

    // Read and advance using the specified timestep.
    // Updates the stored dt_s_ for subsequent read() calls.
    ManualInputFrame read(float dt_s);

    // Propagate SimRunner timestep.
    void setTimestep(float dt_s) override { dt_s_ = dt_s; }

    // Replace the key-state provider (e.g., swap in a mock after construction).
    void setKeyStateProvider(KeyStateProvider provider);

    static KeyStateProvider defaultKeyStateProvider();

private:
    KeyboardInputConfig config_;
    AircraftCommand     command_;
    KeyStateProvider    key_provider_;
    float               dt_s_ = 0.02f;

    // Previous state of action keys for edge detection.
    bool prev_center_    = false;
    bool prev_sim_reset_ = false;
    bool prev_sim_start_ = false;

    void applyKeys(const Uint8* keys, float dt_s, uint32_t& actions_out);
    void clampCommand();
    AircraftCommand neutralCommand() const;
};

}  // namespace liteaero::simulation
