#include "input/ScriptedInput.hpp"

namespace liteaero::simulation {

void ScriptedInput::initialize(const nlohmann::json& /*config*/)
{
    // No configuration parameters. Accept any JSON object (including empty).
    reset();
}

void ScriptedInput::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    frame_ = ManualInputFrame{};
    // Neutral command: level flight, zero lateral/roll, zero throttle.
    frame_.command.n_z               = 1.0f;
    frame_.command.n_y               = 0.0f;
    frame_.command.rollRate_Wind_rps = 0.0f;
    frame_.command.throttle_nd       = 0.0f;
}

ManualInputFrame ScriptedInput::read()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_;
}

void ScriptedInput::push(const AircraftCommand& command)
{
    std::lock_guard<std::mutex> lock(mutex_);
    frame_.command = command;
    frame_.actions = 0u;
}

}  // namespace liteaero::simulation
