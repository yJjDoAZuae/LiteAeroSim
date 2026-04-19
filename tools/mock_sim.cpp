// mock_sim — kinematic mock broadcaster for Godot viewer diagnostics.
//
// Replaces the full aerodynamics engine with a direct joystick→attitude map,
// so the Godot rendering / camera / coordinate chain can be verified without
// aerodynamic model involvement.
//
// Attitude mapping (all linear):
//   nz    → pitch:  1g→0°, 4g→+40°, -1g→-10°   (pull = nose up)
//   roll  → roll:   ±max_roll_rate_rad_s → ±30°
//   ny    → yaw rate: ±max_ny → ±5 deg/s
//   throttle → forward speed: 0→0.1 m/s, 1→1.0 m/s (horizontal only)
//
// Usage (from liteaero-sim root):
//   build\tools\mock_sim.exe --joystick python\gx12_config.json
//                            [--port 14560] [--dt 0.02]
//                            [--lat 0.60073] [--lon -2.09139] [--alt 103.0]

#include "broadcaster/UdpSimulationBroadcaster.hpp"
#include "input/JoystickInput.hpp"
#include "SimulationFrame.hpp"

#include <Eigen/Dense>
#include <SDL2/SDL.h>
#include <nlohmann/json.hpp>

#include <csignal>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static constexpr double k_earth_radius_m = 6371000.0;
static constexpr float  k_deg2rad        = static_cast<float>(M_PI / 180.0);
static constexpr float  k_rad2deg        = static_cast<float>(180.0 / M_PI);

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

static volatile bool g_stop = false;
static void handle_signal(int) { g_stop = true; }

// ---------------------------------------------------------------------------
// Args
// ---------------------------------------------------------------------------

struct Args {
    std::string joystick_config_path;
    uint16_t    port     = 14560;
    float       dt_s     = 0.02f;
    double      lat_rad  = 0.60073;
    double      lon_rad  = -2.09139;
    float       alt_m    = 103.0f;
};

static void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog
              << " --joystick <json>"
                 " [--port <udp-port>] [--dt <s>]"
                 " [--lat <rad>] [--lon <rad>] [--alt <m>]\n";
}

static Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string f(argv[i]);
        if (f == "--joystick" && i + 1 < argc)      a.joystick_config_path = argv[++i];
        else if (f == "--port"  && i + 1 < argc)    a.port    = static_cast<uint16_t>(std::stoi(argv[++i]));
        else if (f == "--dt"    && i + 1 < argc)    a.dt_s    = std::stof(argv[++i]);
        else if (f == "--lat"   && i + 1 < argc)    a.lat_rad = std::stod(argv[++i]);
        else if (f == "--lon"   && i + 1 < argc)    a.lon_rad = std::stod(argv[++i]);
        else if (f == "--alt"   && i + 1 < argc)    a.alt_m   = std::stof(argv[++i]);
        else { std::cerr << "Unknown argument: " << f << "\n"; print_usage(argv[0]); std::exit(1); }
    }
    if (a.joystick_config_path.empty()) {
        std::cerr << "Error: --joystick is required\n";
        print_usage(argv[0]);
        std::exit(1);
    }
    return a;
}

// ---------------------------------------------------------------------------
// Attitude → q_nb  (ZYX: heading, pitch, roll)
// ---------------------------------------------------------------------------

static Eigen::Quaternionf euler_zyx_to_qnb(float heading_rad, float pitch_rad, float roll_rad) {
    return (Eigen::AngleAxisf(heading_rad, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(pitch_rad,   Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(roll_rad,    Eigen::Vector3f::UnitX())).normalized();
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);

    const Args args = parse_args(argc, argv);

    // --- SDL joystick init ---
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    // --- Load joystick config ---
    std::ifstream joy_file(args.joystick_config_path);
    if (!joy_file.is_open()) {
        std::cerr << "Cannot open joystick config: " << args.joystick_config_path << "\n";
        SDL_Quit();
        return 1;
    }
    nlohmann::json joy_json;
    try { joy_file >> joy_json; }
    catch (const std::exception& e) {
        std::cerr << "Joystick JSON parse error: " << e.what() << "\n";
        SDL_Quit();
        return 1;
    }

    JoystickInput joystick(0);
    try { joystick.initialize(joy_json); }
    catch (const std::exception& e) {
        std::cerr << "JoystickInput init failed: " << e.what() << "\n";
        SDL_Quit();
        return 1;
    }

    // --- Broadcaster ---
    UdpSimulationBroadcaster broadcaster(args.port);
    std::cout << "mock_sim broadcasting to 127.0.0.1:" << args.port << "\n";
    std::cout << "Press Ctrl+C to stop.\n";

    // --- State ---
    double lat_rad     = args.lat_rad;
    double lon_rad     = args.lon_rad;
    float  alt_m       = args.alt_m;
    float  heading_rad = 0.0f;   // NED heading (rad), 0 = north
    float  pitch_rad   = 0.0f;
    float  roll_rad    = 0.0f;
    double sim_time_s  = 0.0;

    const float dt = args.dt_s;

    // Joystick config limits — read from config for consistent scaling.
    const float max_nz          = joy_json.value("max_nz_g",           4.0f);
    const float min_nz          = joy_json.value("min_nz_g",          -2.0f);  // actually min_nz_g
    const float max_roll_rad_s  = joy_json.value("max_roll_rate_rad_s", static_cast<float>(M_PI / 2.0));

    // Attitude mapping parameters.
    // nz=1 → 0°, nz=max_nz → +40°, nz=min_nz → -10°
    // Positive nz above 1 maps linearly to positive pitch (pull = nose up).
    // nz below 1 maps to negative pitch down to -10°.
    const float pitch_per_nz_pos = 40.0f * k_deg2rad / (max_nz - 1.0f);  // rad per g above 1
    const float pitch_per_nz_neg = 10.0f * k_deg2rad / (1.0f - min_nz);  // rad per g below 1

    // Roll: full roll stick → ±30°
    const float roll_per_rate    = 30.0f * k_deg2rad / max_roll_rad_s;

    // Yaw: max ny_g → ±5 deg/s
    const float max_ny_g         = joy_json.value("max_ny_g", 1.0f);
    const float yaw_rate_per_ny  = 5.0f * k_deg2rad / max_ny_g;   // rad/s per g

    using Clock    = std::chrono::steady_clock;
    using Duration = std::chrono::duration<double>;

    const auto t_start = Clock::now();

    while (!g_stop) {
        SDL_PumpEvents();
        const ManualInputFrame frame = joystick.read();
        const AircraftCommand& cmd   = frame.command;

        // --- Attitude update ---

        // Pitch: direct map from nz, no integration.
        if (cmd.n_z >= 1.0f)
            pitch_rad = (cmd.n_z - 1.0f) * pitch_per_nz_pos;
        else
            pitch_rad = -(1.0f - cmd.n_z) * pitch_per_nz_neg;

        // Roll: direct map from roll rate command.
        roll_rad = cmd.rollRate_Wind_rps * roll_per_rate;

        // Heading: integrate yaw rate from ny.
        heading_rad += cmd.n_y * yaw_rate_per_ny * dt;
        // Wrap to [-π, +π].
        while (heading_rad >  static_cast<float>(M_PI)) heading_rad -= 2.0f * static_cast<float>(M_PI);
        while (heading_rad < -static_cast<float>(M_PI)) heading_rad += 2.0f * static_cast<float>(M_PI);

        // --- Position update ---
        // Speed in horizontal plane only, in nose direction.
        // throttle ∈ [0, 1] → speed ∈ [0.1, 1.0] m/s
        const float speed_mps = 0.1f + cmd.throttle_nd * 0.9f;
        // Horizontal nose direction in NED: heading_rad measured from North.
        const float v_north = speed_mps * std::cos(heading_rad);
        const float v_east  = speed_mps * std::sin(heading_rad);

        const double lat_rate = v_north / k_earth_radius_m;
        const double lon_rate = v_east  / (k_earth_radius_m * std::cos(lat_rad));
        lat_rad += lat_rate * static_cast<double>(dt);
        lon_rad += lon_rate * static_cast<double>(dt);
        // Altitude stays fixed — no vertical motion from throttle.

        // --- Build q_nb ---
        const Eigen::Quaternionf q_nb = euler_zyx_to_qnb(heading_rad, pitch_rad, roll_rad);

        // --- Broadcast ---
        SimulationFrame sf{};
        sf.timestamp_s       = sim_time_s;
        sf.latitude_rad      = lat_rad;
        sf.longitude_rad     = lon_rad;
        sf.height_wgs84_m    = alt_m;
        sf.q_w               = q_nb.w();
        sf.q_x               = q_nb.x();
        sf.q_y               = q_nb.y();
        sf.q_z               = q_nb.z();
        sf.velocity_north_mps = v_north;
        sf.velocity_east_mps  = v_east;
        sf.velocity_down_mps  = 0.0f;
        broadcaster.broadcast(sf);

        sim_time_s += static_cast<double>(dt);

        // --- Real-time pacing ---
        const auto t_target = t_start + std::chrono::duration_cast<Clock::duration>(
            Duration(sim_time_s));
        std::this_thread::sleep_until(t_target);
    }

    SDL_Quit();
    std::cout << "mock_sim stopped after " << sim_time_s << " s.\n";
    return 0;
}
