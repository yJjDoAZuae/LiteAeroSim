// SimulationReceiver.cpp — GDExtension UDP receiver implementation.
//
// Design authority: docs/architecture/godot_plugin.md §SimulationReceiver Class
//
// Receives SimulationFrameProto datagrams from live_sim and drives the parent
// Vehicle node transform each render frame via linear/slerp interpolation between
// the two most recently received sim frames.
//
// Coordinate mapping (design authority: live_sim_view.md §Coordinate System):
//   ENU -> Godot:  X = East, Y = Up, Z = -North
//   NED -> Godot rotation quaternion: Quaternion(x=0.5, y=0.5, z=-0.5, w=0.5)

#include "SimulationReceiver.hpp"
#include "liteaerosim.pb.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <cmath>
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef int socklen_t;
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR   (-1)
#define closesocket    close
#endif

using namespace godot;

namespace {
    constexpr double k_earth_radius_m = 6371000.0;
    constexpr int    k_recv_buf_size  = 512;
} // namespace

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

SimulationReceiver::SimulationReceiver() = default;

SimulationReceiver::~SimulationReceiver() {
    _close_socket();
}

// ---------------------------------------------------------------------------
// ClassDB binding
// ---------------------------------------------------------------------------

void SimulationReceiver::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_broadcast_port", "port"),
                         &SimulationReceiver::set_broadcast_port);
    ClassDB::bind_method(D_METHOD("get_broadcast_port"),
                         &SimulationReceiver::get_broadcast_port);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "broadcast_port"),
                 "set_broadcast_port", "get_broadcast_port");

    ClassDB::bind_method(D_METHOD("set_max_datagrams_per_frame", "n"),
                         &SimulationReceiver::set_max_datagrams_per_frame);
    ClassDB::bind_method(D_METHOD("get_max_datagrams_per_frame"),
                         &SimulationReceiver::get_max_datagrams_per_frame);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "max_datagrams_per_frame"),
                 "set_max_datagrams_per_frame", "get_max_datagrams_per_frame");

    ClassDB::bind_method(
        D_METHOD("set_world_origin", "lat_rad", "lon_rad", "h_m"),
        &SimulationReceiver::set_world_origin);
}

// ---------------------------------------------------------------------------
// Property accessors
// ---------------------------------------------------------------------------

void SimulationReceiver::set_broadcast_port(int port) { broadcast_port_ = port; }
int  SimulationReceiver::get_broadcast_port() const   { return broadcast_port_; }

void SimulationReceiver::set_max_datagrams_per_frame(int n) { max_datagrams_per_frame_ = n; }
int  SimulationReceiver::get_max_datagrams_per_frame() const { return max_datagrams_per_frame_; }

void SimulationReceiver::set_world_origin(double lat_rad, double lon_rad, double h_m) {
    world_origin_lat_rad_ = lat_rad;
    world_origin_lon_rad_ = lon_rad;
    world_origin_h_m_     = h_m;
    world_origin_set_     = true;
}

// ---------------------------------------------------------------------------
// Godot lifecycle
// ---------------------------------------------------------------------------

void SimulationReceiver::_ready() {
    if (Engine::get_singleton()->is_editor_hint())
        return;
    _open_socket();
}

void SimulationReceiver::_process(double /*delta*/) {
    if (Engine::get_singleton()->is_editor_hint())
        return;
    if (socket_fd_ < 0)
        return;

    // Drain queued datagrams — each decoded frame shifts curr→prev and
    // stores the new frame in curr, so after the loop frame_prev_ and
    // frame_curr_ are the two most recent sim frames.
    uint8_t buf[k_recv_buf_size];
    int datagrams = 0;
    while (datagrams < max_datagrams_per_frame_) {
        int n = static_cast<int>(
            recvfrom(socket_fd_, reinterpret_cast<char*>(buf),
                     static_cast<int>(sizeof(buf)), 0, nullptr, nullptr));
        if (n <= 0)
            break;
        _decode_frame(buf, n);
        ++datagrams;
    }

    if (!frame_curr_.valid)
        return;

    // Interpolate between frame_prev_ and frame_curr_ based on current
    // wall time.  t=0 → frame_prev_, t=1 → frame_curr_, t>1 extrapolates
    // (clamped to 1 to avoid runaway when no new packets arrive).
    double t = 1.0;
    if (frame_prev_.valid) {
        const double dt_frames = frame_curr_.wall_time_s - frame_prev_.wall_time_s;
        if (dt_frames > 1e-6) {
            const double now_s = static_cast<double>(
                Time::get_singleton()->get_ticks_usec()) * 1e-6;
            t = (now_s - frame_prev_.wall_time_s) / dt_frames;
            // Clamp: never extrapolate past curr, allow up to 1 frame behind.
            t = Math::clamp(t, 0.0, 1.0);
        }
    }

    const Vector3    pos = frame_prev_.valid
        ? frame_prev_.position.lerp(frame_curr_.position, static_cast<float>(t))
        : frame_curr_.position;
    const Quaternion rot = frame_prev_.valid
        ? frame_prev_.rotation.slerp(frame_curr_.rotation, static_cast<float>(t))
        : frame_curr_.rotation;

    get_parent()->set("position",   pos);
    get_parent()->set("quaternion", rot);
}

// ---------------------------------------------------------------------------
// Socket helpers
// ---------------------------------------------------------------------------

void SimulationReceiver::_open_socket() {
    socket_fd_ = static_cast<int>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    if (socket_fd_ == INVALID_SOCKET) {
        ERR_PRINT("SimulationReceiver: socket() failed");
        socket_fd_ = -1;
        return;
    }

    // Allow immediate rebind after Godot restarts (avoids EADDRINUSE / Error 10048).
    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&reuse), sizeof(reuse));

    // Bind to loopback so only local traffic is received.
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(broadcast_port_));
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        ERR_PRINT(String("SimulationReceiver: bind() failed on port ") +
                  String::num_int64(broadcast_port_));
        closesocket(socket_fd_);
        socket_fd_ = -1;
        return;
    }

    // Set non-blocking so _process() returns immediately when no data arrives.
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(socket_fd_, FIONBIO, &mode);
#else
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
#endif
}

void SimulationReceiver::_close_socket() {
    if (socket_fd_ >= 0) {
        closesocket(socket_fd_);
        socket_fd_ = -1;
    }
}

// ---------------------------------------------------------------------------
// Frame decoding
// ---------------------------------------------------------------------------

void SimulationReceiver::_decode_frame(const uint8_t* data, int size) {
    las_proto::SimulationFrameProto frame;
    if (!frame.ParseFromArray(data, size)) {
        UtilityFunctions::print("SimulationReceiver: proto parse failed, size=", size);
        return;
    }

    if (!world_origin_set_) {
        UtilityFunctions::print("SimulationReceiver: world origin not set — dropping frame");
        return;
    }

    // Geodetic -> ENU offset from world origin (flat-Earth approximation).
    double dlat    = frame.latitude_rad()  - world_origin_lat_rad_;
    double dlon    = frame.longitude_rad() - world_origin_lon_rad_;
    double north_m = dlat * k_earth_radius_m;
    double east_m  = dlon * k_earth_radius_m * std::cos(world_origin_lat_rad_);
    double up_m    = static_cast<double>(frame.height_wgs84_m()) - world_origin_h_m_;

    // ENU -> Godot position: X=East, Y=Up, Z=-North.
    const Vector3 pos(static_cast<float>(east_m),
                      static_cast<float>(up_m),
                      static_cast<float>(-north_m));

    // Body-to-NED quaternion -> body-to-Godot quaternion.
    // NED->Godot frame rotation: Quaternion(x=0.5, y=0.5, z=-0.5, w=0.5)
    const Quaternion r_ned_to_godot(0.5f, 0.5f, -0.5f, 0.5f);
    Quaternion q_b2n(frame.q_x(), frame.q_y(), frame.q_z(), frame.q_w());
    q_b2n = q_b2n.normalized();
    const Quaternion rot = (r_ned_to_godot * q_b2n).normalized();

    // Shift curr -> prev, store new frame in curr.
    frame_prev_ = frame_curr_;
    frame_curr_.position    = pos;
    frame_curr_.rotation    = rot;
    frame_curr_.wall_time_s = static_cast<double>(
        Time::get_singleton()->get_ticks_usec()) * 1e-6;
    frame_curr_.valid       = true;
}
