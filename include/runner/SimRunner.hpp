#pragma once

#include "Aircraft.hpp"

#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <thread>

namespace liteaero::simulation {

enum class ExecutionMode { RealTime, ScaledRealTime, Batch };

struct RunnerConfig {
    ExecutionMode mode       = ExecutionMode::Batch;
    float         dt_s       = 0.02f;
    float         time_scale = 1.0f;
    double        duration_s = 0.0;
};

class SimRunner {
public:
    ~SimRunner();

    void   initialize(const RunnerConfig& config, Aircraft& aircraft);
    void   start();
    void   stop();
    bool   is_running() const;
    double elapsed_sim_time_s() const;

private:
    void runLoop();

    RunnerConfig          config_;
    Aircraft*             aircraft_   = nullptr;
    std::atomic<bool>     stop_flag_  {false};
    std::atomic<bool>     running_    {false};
    std::atomic<uint64_t> step_count_ {0};
    std::thread           worker_;
};

}  // namespace liteaero::simulation
