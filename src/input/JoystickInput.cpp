#include "input/JoystickInput.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Apply midpoint trim when raw_trim is 0 and midpoint is not 0.
// Per design: raw_trim = 0 in JSON triggers midpoint calculation.
void JoystickInput::applyMidpointTrim(AxisMapping& m)
{
    if (m.raw_trim == 0) {
        const int mid = (static_cast<int>(m.raw_min) + static_cast<int>(m.raw_max)) / 2;
        m.raw_trim = static_cast<int16_t>(mid);
    }
}

// ---------------------------------------------------------------------------
// Constructors / destructor
// ---------------------------------------------------------------------------

JoystickInput::JoystickInput(int device_index)
{
    // Production path: open the SDL device.
    // Device selection by name or index is applied in initialize().
    // Store device_index as default; name-based selection overrides in initialize().
    config_.device_index = device_index;
    // joystick_ opened in initialize() once config is known.
}

JoystickInput::JoystickInput(AxisProvider provider)
    : axis_provider_(std::move(provider))
{
    // Test path: no SDL device opened.
    connected_ = true;
}

JoystickInput::~JoystickInput()
{
    if (joystick_) {
        SDL_JoystickClose(joystick_);
        joystick_ = nullptr;
    }
}

// ---------------------------------------------------------------------------
// initialize()
// ---------------------------------------------------------------------------

static void parseAxisMapping(const nlohmann::json& j, AxisMapping& m)
{
    if (j.contains("sdl_axis_index")) m.sdl_axis_index = j["sdl_axis_index"].get<int>();
    if (j.contains("center_output"))  m.center_output  = j["center_output"].get<float>();
    if (j.contains("scale"))          m.scale          = j["scale"].get<float>();
    if (j.contains("inverted"))       m.inverted        = j["inverted"].get<bool>();
    if (j.contains("raw_min"))        m.raw_min        = static_cast<int16_t>(j["raw_min"].get<int>());
    if (j.contains("raw_max"))        m.raw_max        = static_cast<int16_t>(j["raw_max"].get<int>());
    if (j.contains("raw_trim"))       m.raw_trim       = static_cast<int16_t>(j["raw_trim"].get<int>());
}

void JoystickInput::initialize(const nlohmann::json& config)
{
    if (config.contains("dead_zone_nd")) {
        config_.dead_zone_nd = config["dead_zone_nd"].get<float>();
    }
    if (config_.dead_zone_nd >= 1.0f || config_.dead_zone_nd < 0.0f) {
        throw std::invalid_argument("JoystickInputConfig::dead_zone_nd must be in [0, 1)");
    }

    if (config.contains("nz_axis"))       parseAxisMapping(config["nz_axis"],       config_.nz_axis);
    if (config.contains("ny_axis"))       parseAxisMapping(config["ny_axis"],       config_.ny_axis);
    if (config.contains("roll_axis"))     parseAxisMapping(config["roll_axis"],     config_.roll_axis);
    if (config.contains("throttle_axis")) parseAxisMapping(config["throttle_axis"], config_.throttle_axis);

    if (config.contains("min_throttle_nd"))    config_.min_throttle_nd    = config["min_throttle_nd"].get<float>();
    if (config.contains("idle_throttle_nd"))   config_.idle_throttle_nd   = config["idle_throttle_nd"].get<float>();
    if (config.contains("min_nz_g"))           config_.min_nz_g           = config["min_nz_g"].get<float>();
    if (config.contains("max_nz_g"))           config_.max_nz_g           = config["max_nz_g"].get<float>();
    if (config.contains("max_ny_g"))           config_.max_ny_g           = config["max_ny_g"].get<float>();
    if (config.contains("max_roll_rate_rad_s"))config_.max_roll_rate_rad_s= config["max_roll_rate_rad_s"].get<float>();
    if (config.contains("btn_center"))         config_.btn_center         = config["btn_center"].get<int>();
    if (config.contains("btn_capture_trim"))   config_.btn_capture_trim   = config["btn_capture_trim"].get<int>();
    if (config.contains("btn_reset_trim"))     config_.btn_reset_trim     = config["btn_reset_trim"].get<int>();
    if (config.contains("btn_sim_reset"))      config_.btn_sim_reset      = config["btn_sim_reset"].get<int>();
    if (config.contains("btn_sim_start"))      config_.btn_sim_start      = config["btn_sim_start"].get<int>();
    if (config.contains("device_name_contains")) config_.device_name_contains = config["device_name_contains"].get<std::string>();
    if (config.contains("device_index"))       config_.device_index       = config["device_index"].get<int>();

    // Apply midpoint trim for any axis where raw_trim = 0
    applyMidpointTrim(config_.nz_axis);
    applyMidpointTrim(config_.ny_axis);
    applyMidpointTrim(config_.roll_axis);
    applyMidpointTrim(config_.throttle_axis);

    // Save trim state for ResetTrim action
    trim_reset_ = config_;

    // Open SDL device for production path (axis_provider_ not set)
    if (!axis_provider_ && !joystick_) {
        int dev_idx = config_.device_index;
        if (!config_.device_name_contains.empty()) {
            const int n = SDL_NumJoysticks();
            bool found = false;
            for (int i = 0; i < n; ++i) {
                const char* name = SDL_JoystickNameForIndex(i);
                if (name) {
                    std::string s(name);
                    // Case-insensitive substring match
                    std::string lower_s(s), lower_sub(config_.device_name_contains);
                    std::transform(lower_s.begin(), lower_s.end(), lower_s.begin(), ::tolower);
                    std::transform(lower_sub.begin(), lower_sub.end(), lower_sub.begin(), ::tolower);
                    if (lower_s.find(lower_sub) != std::string::npos) {
                        dev_idx = i;
                        found = true;
                        break;
                    }
                }
            }
            if (!found) {
                std::string msg = "JoystickInput: no device name contains '";
                msg += config_.device_name_contains;
                msg += "'. Available devices:";
                for (int i = 0; i < n; ++i) {
                    const char* name = SDL_JoystickNameForIndex(i);
                    msg += "\n  ["; msg += std::to_string(i); msg += "] ";
                    msg += (name ? name : "<unnamed>");
                }
                throw std::runtime_error(msg);
            }
        }
        joystick_ = SDL_JoystickOpen(dev_idx);
        if (!joystick_) {
            throw std::runtime_error(std::string("JoystickInput: SDL_JoystickOpen failed: ")
                                     + SDL_GetError());
        }
    }

    reset();
}

// ---------------------------------------------------------------------------
// reset()
// ---------------------------------------------------------------------------

void JoystickInput::reset()
{
    // Restore trim to the initial configured values
    config_.nz_axis.raw_trim       = trim_reset_.nz_axis.raw_trim;
    config_.ny_axis.raw_trim       = trim_reset_.ny_axis.raw_trim;
    config_.roll_axis.raw_trim     = trim_reset_.roll_axis.raw_trim;
    config_.throttle_axis.raw_trim = trim_reset_.throttle_axis.raw_trim;

    prev_btn_center_       = false;
    prev_btn_capture_trim_ = false;
    prev_btn_reset_trim_   = false;
    prev_btn_sim_reset_    = false;
    prev_btn_sim_start_    = false;
}

// ---------------------------------------------------------------------------
// isConnected()
// ---------------------------------------------------------------------------

bool JoystickInput::isConnected() const
{
    return connected_;
}

// ---------------------------------------------------------------------------
// readRawAxis() / readButton()
// ---------------------------------------------------------------------------

Sint16 JoystickInput::readRawAxis(int axis_index) const
{
    if (axis_provider_) {
        return axis_provider_(axis_index);
    }
    if (joystick_) {
        return SDL_JoystickGetAxis(joystick_, axis_index);
    }
    return Sint16(0);
}

bool JoystickInput::readButton(int btn_index) const
{
    if (btn_index < 0)     return false;
    if (axis_provider_)    return false;  // no button support in test path
    if (!joystick_)        return false;
    return SDL_JoystickGetButton(joystick_, btn_index) != 0;
}

// ---------------------------------------------------------------------------
// checkDisconnectEvents()
// ---------------------------------------------------------------------------

void JoystickInput::checkDisconnectEvents()
{
    if (axis_provider_) return;  // injected provider never disconnects
    if (!connected_)    return;

    SDL_Event ev;
    while (SDL_PeepEvents(&ev, 1, SDL_GETEVENT, SDL_JOYDEVICEREMOVED,
                          SDL_JOYDEVICEREMOVED) > 0) {
        connected_ = false;
        return;
    }
}

// ---------------------------------------------------------------------------
// captureTrim()
// ---------------------------------------------------------------------------

void JoystickInput::captureTrim()
{
    config_.nz_axis.raw_trim       = readRawAxis(config_.nz_axis.sdl_axis_index);
    config_.ny_axis.raw_trim       = readRawAxis(config_.ny_axis.sdl_axis_index);
    config_.roll_axis.raw_trim     = readRawAxis(config_.roll_axis.sdl_axis_index);
    config_.throttle_axis.raw_trim = readRawAxis(config_.throttle_axis.sdl_axis_index);
}

// ---------------------------------------------------------------------------
// applyAxisPipeline()
// ---------------------------------------------------------------------------

float JoystickInput::applyAxisPipeline(Sint16 raw, const AxisMapping& m, float dead_zone) const
{
    // Step 1: calibrate, trim, and normalize to [-1, +1]
    const float raw_f     = static_cast<float>(raw);
    const float trim_f    = static_cast<float>(m.raw_trim);
    const float half_range = (static_cast<float>(m.raw_max) - static_cast<float>(m.raw_min)) / 2.0f;

    float r = (half_range > 0.0f) ? ((raw_f - trim_f) / half_range) : 0.0f;
    r = std::clamp(r, -1.0f, 1.0f);

    // Step 2: inversion
    if (m.inverted) r = -r;

    // Step 3: dead zone with continuity
    const float abs_r = std::abs(r);
    if (abs_r < dead_zone) {
        r = 0.0f;
    } else {
        r = std::copysign((abs_r - dead_zone) / (1.0f - dead_zone), r);
    }

    // Step 4: map to command units
    return m.center_output + r * m.scale;
}

// ---------------------------------------------------------------------------
// read()
// ---------------------------------------------------------------------------

ManualInputFrame JoystickInput::read()
{
    checkDisconnectEvents();

    if (!connected_) {
        // Disconnect fallback: neutral command
        ManualInputFrame frame;
        frame.command.n_z               = 1.0f;
        frame.command.n_y               = 0.0f;
        frame.command.rollRate_Wind_rps = 0.0f;
        frame.command.throttle_nd       = config_.idle_throttle_nd;
        return frame;
    }

    AircraftCommand cmd;
    cmd.n_z               = applyAxisPipeline(readRawAxis(config_.nz_axis.sdl_axis_index),
                                               config_.nz_axis, config_.dead_zone_nd);
    cmd.n_y               = applyAxisPipeline(readRawAxis(config_.ny_axis.sdl_axis_index),
                                               config_.ny_axis, config_.dead_zone_nd);
    cmd.rollRate_Wind_rps = applyAxisPipeline(readRawAxis(config_.roll_axis.sdl_axis_index),
                                               config_.roll_axis, config_.dead_zone_nd);
    cmd.throttle_nd       = applyAxisPipeline(readRawAxis(config_.throttle_axis.sdl_axis_index),
                                               config_.throttle_axis, config_.dead_zone_nd);

    // Clamp
    cmd.n_z               = std::clamp(cmd.n_z,  config_.min_nz_g,  config_.max_nz_g);
    cmd.n_y               = std::clamp(cmd.n_y, -config_.max_ny_g,  config_.max_ny_g);
    cmd.rollRate_Wind_rps = std::clamp(cmd.rollRate_Wind_rps,
                                       -config_.max_roll_rate_rad_s, config_.max_roll_rate_rad_s);
    cmd.throttle_nd       = std::clamp(cmd.throttle_nd, config_.min_throttle_nd, 1.0f);

    // --- Button actions (edge-triggered) ---
    uint32_t actions = 0u;

    auto edge = [&](int btn_idx, bool& prev, InputAction action) {
        const bool now = readButton(btn_idx);
        if (now && !prev) actions |= static_cast<uint32_t>(action);
        prev = now;
    };

    edge(config_.btn_center,       prev_btn_center_,       InputAction::Center);
    edge(config_.btn_capture_trim, prev_btn_capture_trim_, InputAction::CaptureTrim);
    edge(config_.btn_reset_trim,   prev_btn_reset_trim_,   InputAction::ResetTrim);
    edge(config_.btn_sim_reset,    prev_btn_sim_reset_,    InputAction::SimReset);
    edge(config_.btn_sim_start,    prev_btn_sim_start_,    InputAction::SimStart);

    // Center action: snap to neutral
    if (actions & static_cast<uint32_t>(InputAction::Center)) {
        cmd.n_z               = 1.0f;
        cmd.n_y               = 0.0f;
        cmd.rollRate_Wind_rps = 0.0f;
        cmd.throttle_nd       = config_.idle_throttle_nd;
    }

    // CaptureTrim action
    if (actions & static_cast<uint32_t>(InputAction::CaptureTrim)) {
        captureTrim();
    }

    // ResetTrim action
    if (actions & static_cast<uint32_t>(InputAction::ResetTrim)) {
        config_.nz_axis.raw_trim       = trim_reset_.nz_axis.raw_trim;
        config_.ny_axis.raw_trim       = trim_reset_.ny_axis.raw_trim;
        config_.roll_axis.raw_trim     = trim_reset_.roll_axis.raw_trim;
        config_.throttle_axis.raw_trim = trim_reset_.throttle_axis.raw_trim;
    }

    ManualInputFrame frame;
    frame.command = cmd;
    frame.actions = actions;
    return frame;
}

// ---------------------------------------------------------------------------
// enumerateDevices()
// ---------------------------------------------------------------------------

std::vector<JoystickDeviceInfo> JoystickInput::enumerateDevices()
{
    std::vector<JoystickDeviceInfo> result;
    const int n = SDL_NumJoysticks();
    result.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        JoystickDeviceInfo info;
        info.device_index = i;
        const char* name = SDL_JoystickNameForIndex(i);
        info.name = name ? name : "<unnamed>";
        // Open briefly to query axis count
        SDL_Joystick* js = SDL_JoystickOpen(i);
        info.num_axes = js ? SDL_JoystickNumAxes(js) : 0;
        if (js) SDL_JoystickClose(js);
        result.push_back(std::move(info));
    }
    return result;
}

}  // namespace liteaero::simulation
