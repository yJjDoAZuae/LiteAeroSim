# SimRunner — Architecture and Interface Design

This document is the design authority for the simulation runner subsystem. It covers the
execution mode model, threading contract, timing mechanism, and the interface between
`SimRunner` and `Aircraft`.

---

## Use Case Decomposition

| ID | Use Case | Primary Actor | Mechanism |
| --- | --- | --- | --- |
| UC-1 | Run a fixed-duration batch simulation at full rate | Test harness, Monte Carlo, CI | `start()` blocks; returns after `duration_s` steps |
| UC-2 | Run a real-time simulation paced to wall clock | External interface, SITL | `start()` spawns thread; `stop()` or `duration_s` terminates |
| UC-3 | Run a scaled-real-time simulation | Development / slow-motion replay | Same as UC-2 with `time_scale` applied to sleep target |
| UC-4 | Stop a running simulation from another thread | External interface, test | `stop()` sets atomic flag; loop exits within one timestep |
| UC-5 | Query elapsed simulation time | Logger, status display | `elapsed_sim_time_s()` returns `step_count * dt_s` |
| UC-6 | Query whether the runner is currently active | External interface, test | `is_running()` returns atomic flag |

---

## Class Design

### `ExecutionMode`

```cpp
enum class ExecutionMode { RealTime, ScaledRealTime, Batch };
```

| Value | Behavior |
| --- | --- |
| `Batch` | Steps execute as fast as possible. `start()` blocks the caller until `duration_s` elapses or `stop()` is called. |
| `RealTime` | Each step is paced so that the wall time between step starts equals `dt_s`. `start()` spawns a background thread and returns immediately. |
| `ScaledRealTime` | Same as `RealTime` but the wall-time target per step is `dt_s / time_scale`. A `time_scale` of 2.0 runs twice as fast as real time; 0.5 runs at half speed. |

### `RunnerConfig`

```cpp
struct RunnerConfig {
    ExecutionMode mode        = ExecutionMode::Batch;
    float         dt_s        = 0.01f;  // simulation timestep, seconds
    float         time_scale  = 1.0f;   // ScaledRealTime only; must be > 0
    double        duration_s  = 0.0;    // 0 = run until stop() is called
};
```

`dt_s` is owned by `RunnerConfig` rather than `Aircraft` because the runner is responsible
for the wall-clock relationship. `Aircraft::step()` is stateless with respect to timestep
size; the caller supplies it.

### `SimRunner`

```cpp
class SimRunner {
public:
    void   initialize(const RunnerConfig& config, Aircraft& aircraft);
    void   start();
    void   stop();
    bool   is_running() const;
    double elapsed_sim_time_s() const;
};
```

`SimRunner` holds a non-owning reference to `Aircraft`. The caller is responsible for
keeping `Aircraft` alive for the duration of the run.

---

## Threading Contract

| Mode | `start()` behavior | Thread ownership |
| --- | --- | --- |
| `Batch` | Blocks. Returns when the run completes. | No thread spawned. |
| `RealTime` / `ScaledRealTime` | Returns immediately after spawning a `std::thread`. | `SimRunner` owns the thread; joined by `stop()` or destructor. |

`stop()` sets an `std::atomic<bool>` stop flag and, for threaded modes, joins the worker
thread before returning. Calling `stop()` from the run thread itself (e.g., from within a
callback) is undefined behavior.

`is_running()` and `elapsed_sim_time_s()` are safe to call from any thread. `is_running()`
reads the same atomic flag. `elapsed_sim_time_s()` is derived from
`step_count_.load() * config_.dt_s`, where `step_count_` is `std::atomic<uint64_t>`.

`initialize()` must not be called while a run is in progress.

---

## Timing Mechanism — RealTime and ScaledRealTime

The wall-time target for step $k$ is:

$$t_{\text{wall},k} = t_{\text{start}} + k \cdot \frac{dt\_s}{\text{time\_scale}}$$

where $t_{\text{start}}$ is the `std::chrono::steady_clock` value captured at the first
step.

After each step, the runner computes the remaining sleep duration:

$$\Delta t_{\text{sleep}} = t_{\text{wall},k} - t_{\text{now}}$$

If $\Delta t_{\text{sleep}} > 0$, the runner calls `std::this_thread::sleep_until(t_{\text{wall},k})`.

**Late-step policy.** If a step takes longer than the wall-time budget ($\Delta t_{\text{sleep}} \leq 0$), the runner does not attempt to compensate by skipping steps or shortening future sleep intervals. It simply proceeds immediately to the next step, accepting the accumulated lag. This keeps the simulation time monotonic and the step count exact.

The `Batch` mode has no timing mechanism; `sleep_until` is never called.

---

## Private Members

```cpp
private:
    RunnerConfig            config_;
    Aircraft*               aircraft_  = nullptr;
    std::atomic<bool>       stop_flag_ {false};
    std::atomic<bool>       running_   {false};
    std::atomic<uint64_t>   step_count_{0};
    std::thread             worker_;

    void runLoop();
```

`runLoop()` contains the step-advance logic common to all modes and is called either
directly (Batch) or from the worker thread.

---

## `initialize()` Contract

`initialize()` must be called exactly once before `start()`. It:

1. Validates `config_.dt_s > 0`.
2. Validates `config_.time_scale > 0`.
3. Validates `config_.duration_s >= 0`.
4. Stores `config_` and a pointer to `aircraft_`.
5. Resets `step_count_` to 0, `stop_flag_` to false.

Calling `initialize()` on a running instance throws `std::logic_error`.

---

## CMake Integration

```cmake
# src/CMakeLists.txt (addition)
add_library(liteaero_sim_runner STATIC
    runner/SimRunner.cpp
)
add_library(liteaerosim::runner ALIAS liteaero_sim_runner)
target_include_directories(liteaero_sim_runner
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(liteaero_sim_runner PUBLIC cxx_std_17)
target_link_libraries(liteaero_sim_runner
    PUBLIC
        liteaero_sim_aircraft   # provides Aircraft
)
```

`SimRunner` has no dependency on logging, serialization, or flight components.

---

## Test Strategy

File: `test/SimRunner_test.cpp`

| Test | Pass criterion |
| --- | --- |
| `BatchMode_StepCountExact` | `Batch` with `duration_s = 1.0`, `dt_s = 0.01`: `Aircraft::step()` called exactly 100 times; `elapsed_sim_time_s()` returns 1.0 |
| `BatchMode_ZeroDuration_RunsUntilStop` | `duration_s = 0` with `stop()` called after 5 steps from a second thread: run terminates; step count is 5 ± 1 |
| `RealTime_WallTimeAccuracy` | `RealTime` with `dt_s = 0.02`: wall time elapsed after 10 steps is within 15% of 0.2 s |
| `ScaledRealTime_HalfSpeed` | `ScaledRealTime` with `dt_s = 0.02`, `time_scale = 0.5`: wall time elapsed after 5 steps is within 15% of 0.2 s (twice as slow) |
| `Stop_TerminatesWithinOneStep` | `RealTime` runner stopped from external thread: `is_running()` returns false within `2 * dt_s` of `stop()` call |
| `ElapsedSimTime_AccumulatesCorrectly` | After $n$ steps in `Batch` mode, `elapsed_sim_time_s()` equals $n \cdot dt\_s$ exactly |
| `Initialize_InvalidDt_Throws` | `dt_s <= 0` causes `initialize()` to throw |
| `Initialize_InvalidTimeScale_Throws` | `time_scale <= 0` causes `initialize()` to throw |
| `IsRunning_FalseBeforeStart` | `is_running()` is false before `start()` |
| `IsRunning_FalseAfterBatchCompletes` | `is_running()` is false after a `Batch` run completes |
