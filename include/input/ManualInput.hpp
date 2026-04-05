#pragma once

// Design authority: docs/architecture/manual_input.md

#include "Aircraft.hpp"
#include <nlohmann/json.hpp>
#include <cstdint>

namespace liteaero::simulation {

// Named actions that ManualInput can report, independent of which physical
// control triggered them.  Action bits are edge-triggered: a bit is set only
// on the tick the button or key transitions from not-pressed to pressed.
enum class InputAction : uint32_t {
    Center      = 1u << 0,  // snap all command axes to neutral immediately
    CaptureTrim = 1u << 1,  // set current raw axis positions as the trim reference
    ResetTrim   = 1u << 2,  // restore trim to the configured initial values
    SimReset    = 1u << 3,  // signal: application should reset the simulation
    SimStart    = 1u << 4,  // signal: application should start or unpause the run
};

struct ManualInputFrame {
    AircraftCommand command;
    uint32_t        actions = 0u;  // bitmask of InputAction flags active this tick
};

inline bool hasAction(const ManualInputFrame& frame, InputAction action) {
    return (frame.actions & static_cast<uint32_t>(action)) != 0u;
}

// Abstract base for all manual control adapters.
//
// Lifecycle: subclass(...) → initialize(config) → reset() → read() [×N]
class ManualInput {
public:
    // Initialize from a JSON config object specific to the concrete subclass.
    // Throws std::invalid_argument on missing or out-of-range fields.
    virtual void initialize(const nlohmann::json& config) = 0;

    // Reset command state and trim to configured initial values.
    virtual void reset() = 0;

    // Return the AircraftCommand and any fired InputActions for this tick.
    // Non-blocking.  Must be called from the simulation thread.
    virtual ManualInputFrame read() = 0;

    // Set the integration timestep used by adapters that integrate over time
    // (KeyboardInput).  No-op for adapters that do not require a timestep.
    // Called by SimRunner::setManualInput() to propagate the runner dt_s.
    virtual void setTimestep(float /*dt_s*/) {}

    virtual ~ManualInput() = default;
};

}  // namespace liteaero::simulation
