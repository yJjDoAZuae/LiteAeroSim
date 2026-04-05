#pragma once

// Design authority: docs/architecture/manual_input.md

#include "input/ManualInput.hpp"
#include <SDL2/SDL.h>
#include <functional>
#include <string>
#include <vector>

namespace liteaero::simulation {

struct AxisMapping {
    int     sdl_axis_index = 0;
    float   center_output  = 0.0f;
    float   scale          = 1.0f;
    bool    inverted       = false;
    int16_t raw_min        = -32768;
    int16_t raw_max        =  32767;
    int16_t raw_trim       = 0;
    // raw_trim = 0 in config causes initialize() to compute the midpoint
    // (raw_min + raw_max) / 2 as the initial trim.
};

struct JoystickInputConfig {
    float dead_zone_nd        = 0.05f;

    AxisMapping nz_axis       = {1, 1.0f,   3.0f,    true,  -32768, 32767, 0};
    AxisMapping ny_axis       = {3, 0.0f,   1.0f,    false, -32768, 32767, 0};
    AxisMapping roll_axis     = {0, 0.0f,   1.5708f, false, -32768, 32767, 0};
    AxisMapping throttle_axis = {2, 0.5f,   0.5f,    true,  -32768, 32767, 0};

    float min_throttle_nd     =  0.0f;
    float idle_throttle_nd    =  0.05f;

    float min_nz_g            = -2.0f;
    float max_nz_g            =  4.0f;
    float max_ny_g            =  2.0f;
    float max_roll_rate_rad_s =  1.5708f;

    // Button indices for named InputActions; -1 = disabled.
    int   btn_center          =  0;
    int   btn_capture_trim    = -1;
    int   btn_reset_trim      = -1;
    int   btn_sim_reset       = -1;
    int   btn_sim_start       = -1;

    // Device selection: non-empty string triggers name-based selection.
    std::string device_name_contains;
    int         device_index = 0;
};

struct JoystickDeviceInfo {
    int         device_index;
    std::string name;
    int         num_axes;
};

class JoystickInput final : public ManualInput {
public:
    // Provider type for injecting raw axis values in unit tests.
    // Returns Sint16 for the given SDL axis index.  When set, no SDL
    // device is opened and SDL event processing is skipped.
    using AxisProvider = std::function<Sint16(int axis_index)>;

    // Production constructor: selects device by name or index per config.
    // Throws std::runtime_error if the device cannot be opened.
    // SDL_Init(SDL_INIT_JOYSTICK) must have been called before construction.
    explicit JoystickInput(int device_index = 0);

    // Test constructor: uses an injected axis provider; no SDL device opened.
    explicit JoystickInput(AxisProvider provider);

    ~JoystickInput() override;

    JoystickInput(const JoystickInput&)            = delete;
    JoystickInput& operator=(const JoystickInput&) = delete;
    JoystickInput(JoystickInput&&)                 = delete;
    JoystickInput& operator=(JoystickInput&&)      = delete;

    void             initialize(const nlohmann::json& config) override;
    void             reset() override;
    ManualInputFrame read() override;

    // Sets raw_trim on all four axis mappings to the current raw axis readings.
    void             captureTrim();

    bool             isConnected() const;

    // Test-only: force the connected_ flag to false to simulate a disconnect.
    void             simulateDisconnect() { connected_ = false; }

    // Returns info for all enumerated SDL joystick devices.
    // SDL_Init(SDL_INIT_JOYSTICK) must have been called before this.
    static std::vector<JoystickDeviceInfo> enumerateDevices();

private:
    SDL_Joystick*       joystick_   = nullptr;
    AxisProvider        axis_provider_;
    JoystickInputConfig config_;
    JoystickInputConfig trim_reset_;  // copy at initialize() time; used by ResetTrim
    bool                connected_  = true;

    // Previous button states for edge detection (indices match btn_* fields).
    bool prev_btn_center_       = false;
    bool prev_btn_capture_trim_ = false;
    bool prev_btn_reset_trim_   = false;
    bool prev_btn_sim_reset_    = false;
    bool prev_btn_sim_start_    = false;

    float applyAxisPipeline(Sint16 raw, const AxisMapping& mapping, float dead_zone) const;
    void  checkDisconnectEvents();
    bool  readButton(int btn_index) const;
    Sint16 readRawAxis(int axis_index) const;
    static void applyMidpointTrim(AxisMapping& m);
};

}  // namespace liteaero::simulation
