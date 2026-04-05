// joystick_verify — Standalone joystick verification tool.
//
// Design authority: docs/architecture/manual_input.md
//
// Prints a device enumeration table to stdout, then streams ManualInputFrame
// values at ~50 Hz.  Two output formats are supported:
//
//   JSON lines (default) — one JSON object per line; human-readable.
//   Protobuf (--proto)   — length-prefixed binary: uint32 LE byte count
//                          followed by serialized ManualInputFrameProto bytes.
//
// Usage:
//   joystick_verify [--config <json-file>] [--device-index <n>]
//                   [--device-name <substring>] [--rate-hz <hz>]
//                   [--proto]
//
// Output protocol:
//   1. Device enumeration: one line per device, format:
//        DEVICE <index> <num_axes> <name>
//   2. Sentinel line:
//        READY
//   3. Frame stream until EOF or SIGINT.

#include "input/JoystickInput.hpp"
#include "liteaerosim.pb.h"
#include <SDL2/SDL.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

static volatile bool g_stop = false;

static void onSignal(int)
{
    g_stop = true;
}

// ---------------------------------------------------------------------------
// Argument parsing
// ---------------------------------------------------------------------------

struct Options {
    std::string config_path;
    std::string device_name;
    int         device_index = 0;
    float       rate_hz      = 50.0f;
    bool        proto        = false;
};

static void printUsage(const char* prog)
{
    std::cerr << "Usage: " << prog
              << " [--config <json-file>] [--device-index <n>]"
                 " [--device-name <substring>] [--rate-hz <hz>] [--proto]\n";
}

static Options parseArgs(int argc, char* argv[])
{
    Options opts;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--config" && i + 1 < argc) {
            opts.config_path = argv[++i];
        } else if (arg == "--device-index" && i + 1 < argc) {
            opts.device_index = std::stoi(argv[++i]);
        } else if (arg == "--device-name" && i + 1 < argc) {
            opts.device_name = argv[++i];
        } else if (arg == "--rate-hz" && i + 1 < argc) {
            opts.rate_hz = std::stof(argv[++i]);
        } else if (arg == "--proto") {
            opts.proto = true;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            printUsage(argv[0]);
            std::exit(1);
        }
    }
    return opts;
}

// ---------------------------------------------------------------------------
// JSON serialization for ManualInputFrame
// ---------------------------------------------------------------------------

static std::string frameToJsonLine(const liteaero::simulation::ManualInputFrame& frame,
                                   bool connected)
{
    nlohmann::json j;
    j["n_z"]                = frame.command.n_z;
    j["n_y"]                = frame.command.n_y;
    j["roll_rate_wind_rps"] = frame.command.rollRate_Wind_rps;
    j["throttle_nd"]        = frame.command.throttle_nd;
    j["actions"]            = frame.actions;
    j["connected"]          = connected;
    return j.dump();
}

// ---------------------------------------------------------------------------
// Protobuf serialization for ManualInputFrame
// Writes a uint32 LE length prefix followed by serialized proto bytes.
// ---------------------------------------------------------------------------

static void writeProtoFrame(const liteaero::simulation::ManualInputFrame& frame,
                            bool connected)
{
    las_proto::ManualInputFrameProto msg;
    msg.set_schema_version(1);
    msg.set_actions(frame.actions);
    msg.set_connected(connected);

    auto* cmd = msg.mutable_command();
    cmd->set_schema_version(1);
    cmd->set_n_z(frame.command.n_z);
    cmd->set_n_y(frame.command.n_y);
    cmd->set_roll_rate_wind_rps(frame.command.rollRate_Wind_rps);
    cmd->set_throttle_nd(frame.command.throttle_nd);

    const std::string serialized = msg.SerializeAsString();
    const uint32_t byte_count = static_cast<uint32_t>(serialized.size());

    // Write length prefix (little-endian uint32).
    uint8_t prefix[4];
    prefix[0] = static_cast<uint8_t>(byte_count & 0xFFu);
    prefix[1] = static_cast<uint8_t>((byte_count >> 8)  & 0xFFu);
    prefix[2] = static_cast<uint8_t>((byte_count >> 16) & 0xFFu);
    prefix[3] = static_cast<uint8_t>((byte_count >> 24) & 0xFFu);

    std::cout.write(reinterpret_cast<const char*>(prefix), 4);
    std::cout.write(serialized.data(), static_cast<std::streamsize>(serialized.size()));
    std::cout.flush();
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    const Options opts = parseArgs(argc, argv);

    // Install signal handlers for clean shutdown.
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    // SDL initialization — joystick_verify owns the full SDL lifecycle.
    if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_EVENTS) != 0) {
        std::cerr << "joystick_verify: SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    // --- Device enumeration table ---
    const auto devices = liteaero::simulation::JoystickInput::enumerateDevices();
    for (const auto& dev : devices) {
        // Format: DEVICE <index> <num_axes> <name>
        std::cout << "DEVICE " << dev.device_index
                  << " " << dev.num_axes
                  << " " << dev.name << "\n";
    }
    std::cout << "READY\n";
    std::cout.flush();

    if (devices.empty()) {
        std::cerr << "joystick_verify: no joystick devices found. "
                     "Connect a joystick and retry.\n";
        SDL_Quit();
        return 1;
    }

    // --- Build JoystickInput config ---
    nlohmann::json config = nlohmann::json::object();
    if (!opts.config_path.empty()) {
        std::ifstream f(opts.config_path);
        if (!f.is_open()) {
            std::cerr << "joystick_verify: cannot open config file: "
                      << opts.config_path << "\n";
            SDL_Quit();
            return 1;
        }
        try {
            config = nlohmann::json::parse(f);
        } catch (const nlohmann::json::exception& ex) {
            std::cerr << "joystick_verify: config parse error: " << ex.what() << "\n";
            SDL_Quit();
            return 1;
        }
    }

    // Command-line overrides take precedence over config file.
    if (!opts.device_name.empty()) {
        config["device_name_contains"] = opts.device_name;
    } else if (!config.contains("device_name_contains")) {
        config["device_index"] = opts.device_index;
    }

    // --- Construct and initialize JoystickInput ---
    liteaero::simulation::JoystickInput joystick(opts.device_index);
    try {
        joystick.initialize(config);
    } catch (const std::exception& ex) {
        std::cerr << "joystick_verify: initialize failed: " << ex.what() << "\n";
        SDL_Quit();
        return 1;
    }

    // --- Polling loop ---
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(opts.rate_hz)));

    auto t_next = std::chrono::steady_clock::now();

    while (!g_stop) {
        SDL_PumpEvents();

        const liteaero::simulation::ManualInputFrame frame = joystick.read();
        const bool connected = joystick.isConnected();

        if (opts.proto) {
            writeProtoFrame(frame, connected);
        } else {
            std::cout << frameToJsonLine(frame, connected) << "\n";
            std::cout.flush();
        }

        t_next += period;
        const auto t_now = std::chrono::steady_clock::now();
        if (t_next > t_now) {
            std::this_thread::sleep_until(t_next);
        } else {
            // Fell behind; reset pacing reference to avoid spin.
            t_next = t_now;
        }
    }

    SDL_Quit();
    return 0;
}
