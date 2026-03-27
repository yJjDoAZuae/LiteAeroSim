#include "runner/SimRunner.hpp"

#include <chrono>
#include <stdexcept>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------

SimRunner::~SimRunner()
{
    stop_flag_.store(true);
    if (worker_.joinable()) {
        worker_.join();
    }
}

// ---------------------------------------------------------------------------

void SimRunner::initialize(const RunnerConfig& config, Aircraft& aircraft)
{
    if (running_.load()) {
        throw std::logic_error("SimRunner::initialize called while a run is in progress");
    }
    if (config.dt_s <= 0.0f) {
        throw std::invalid_argument("RunnerConfig::dt_s must be > 0");
    }
    if (config.time_scale <= 0.0f) {
        throw std::invalid_argument("RunnerConfig::time_scale must be > 0");
    }
    if (config.duration_s < 0.0) {
        throw std::invalid_argument("RunnerConfig::duration_s must be >= 0");
    }

    config_    = config;
    aircraft_  = &aircraft;
    stop_flag_.store(false);
    step_count_.store(0);
}

// ---------------------------------------------------------------------------

void SimRunner::start()
{
    if (config_.mode == ExecutionMode::Batch) {
        running_.store(true);
        runLoop();
        running_.store(false);
    } else {
        running_.store(true);
        worker_ = std::thread([this] {
            runLoop();
            running_.store(false);
        });
    }
}

// ---------------------------------------------------------------------------

void SimRunner::stop()
{
    stop_flag_.store(true);
    if (worker_.joinable()) {
        worker_.join();
    }
}

// ---------------------------------------------------------------------------

bool SimRunner::is_running() const
{
    return running_.load();
}

// ---------------------------------------------------------------------------

double SimRunner::elapsed_sim_time_s() const
{
    return static_cast<double>(step_count_.load()) * static_cast<double>(config_.dt_s);
}

// ---------------------------------------------------------------------------

void SimRunner::runLoop()
{
    const bool   has_duration   = (config_.duration_s > 0.0);
    const double dt_s_d         = static_cast<double>(config_.dt_s);
    const double time_initial_s = static_cast<double>(step_count_.load()) * dt_s_d;

    const bool timed = (config_.mode == ExecutionMode::RealTime ||
                        config_.mode == ExecutionMode::ScaledRealTime);

    using Clock    = std::chrono::steady_clock;
    using Duration = std::chrono::duration<double>;

    const double wall_step_s = dt_s_d / static_cast<double>(config_.time_scale);

    const auto t_start = Clock::now();

    AircraftCommand cmd{};
    const Eigen::Vector3f wind_NED_mps = Eigen::Vector3f::Zero();
    constexpr float rho_kgm3 = 1.225f;

    while (!stop_flag_.load()) {
        const uint64_t k           = step_count_.load();
        const double   sim_time_s  = static_cast<double>(k) * dt_s_d;

        if (has_duration && sim_time_s > config_.duration_s + time_initial_s) {
            break;
        }

        aircraft_->step(sim_time_s, cmd, wind_NED_mps, rho_kgm3);
        step_count_.store(k + 1);

        if (timed) {
            const auto t_target = t_start + std::chrono::duration_cast<Clock::duration>(
                Duration(static_cast<double>(k + 1) * wall_step_s));
            const auto t_now = Clock::now();
            if (t_target > t_now) {
                std::this_thread::sleep_until(t_target);
            }
        }
    }
}

}  // namespace liteaero::simulation
