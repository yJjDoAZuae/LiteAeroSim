#pragma once

// Design authority: docs/architecture/manual_input.md

#include "input/ManualInput.hpp"
#include <mutex>

namespace liteaero::simulation {

// ManualInput adapter that accepts AircraftCommand values pushed from an
// external thread (e.g. Python via pybind11).  Thread-safe: push() and
// read() are independently protected by a mutex.
//
// Lifecycle: ScriptedInput() → initialize(config) → reset() → read() [×N]
//   push() may be called from any thread at any time after construction.
class ScriptedInput final : public ManualInput {
public:
    ScriptedInput() = default;

    // initialize() accepts an empty JSON config (no parameters required).
    void initialize(const nlohmann::json& config) override;
    void reset() override;

    // Returns the last frame set by push(), or a neutral frame if push() has
    // not been called since the last reset().  Thread-safe.
    ManualInputFrame read() override;

    // Set the command that will be returned by the next read() call.
    // Thread-safe: may be called from a different thread than read().
    void push(const AircraftCommand& command);

private:
    mutable std::mutex mutex_;
    ManualInputFrame   frame_;
};

}  // namespace liteaero::simulation
