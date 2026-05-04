#include "broadcaster/UdpSimulationBroadcaster.hpp"
#include "projection/IViewerProjector.hpp"
#include "liteaerosim.pb.h"

#include <cmath>
#include <cstdio>

#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
   using socket_t = SOCKET;
   static constexpr socket_t kInvalidSocket = INVALID_SOCKET;
   static inline void close_sock(socket_t s) { closesocket(s); }
#else
#  include <sys/socket.h>
#  include <arpa/inet.h>
#  include <unistd.h>
#  include <fcntl.h>
   using socket_t = int;
   static constexpr socket_t kInvalidSocket = -1;
   static inline void close_sock(socket_t s) { close(s); }
#endif

#include <cstring>
#include <string>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------

UdpSimulationBroadcaster::UdpSimulationBroadcaster(
    uint16_t port,
    const liteaero::projection::IViewerProjector* projector)
    : socket_fd_(static_cast<intptr_t>(kInvalidSocket))
    , port_(port)
    , projector_(projector)
{
#ifdef _WIN32
    WSADATA wsa{};
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

    socket_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == kInvalidSocket) return;

    // Set socket to non-blocking so broadcast() never stalls the sim thread.
#ifdef _WIN32
    u_long nonblock = 1;
    ioctlsocket(s, FIONBIO, &nonblock);
#else
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif

    socket_fd_ = static_cast<intptr_t>(s);
}

// ---------------------------------------------------------------------------

UdpSimulationBroadcaster::~UdpSimulationBroadcaster()
{
    socket_t s = static_cast<socket_t>(socket_fd_);
    if (s != kInvalidSocket) {
        close_sock(s);
    }
#ifdef _WIN32
    WSACleanup();
#endif
}

// ---------------------------------------------------------------------------

void UdpSimulationBroadcaster::broadcast(const SimulationFrame& frame)
{
    socket_t s = static_cast<socket_t>(socket_fd_);
    if (s == kInvalidSocket) return;

    las_proto::SimulationFrameProto proto;
    proto.set_timestamp_s(frame.timestamp_s);
    proto.set_latitude_rad(frame.latitude_rad);
    proto.set_longitude_rad(frame.longitude_rad);
    proto.set_height_wgs84_m(frame.height_wgs84_m);
    proto.set_q_w(frame.q_w);
    proto.set_q_x(frame.q_x);
    proto.set_q_y(frame.q_y);
    proto.set_q_z(frame.q_z);
    proto.set_velocity_north_mps(frame.velocity_north_mps);
    proto.set_velocity_east_mps(frame.velocity_east_mps);
    proto.set_velocity_down_mps(frame.velocity_down_mps);
    proto.set_airspeed_mps(frame.airspeed_mps);
    proto.set_agl_m(frame.agl_m);
    proto.set_height_msl_m(frame.height_msl_m);

    // Viewer-projected position (LS-T5 / OQ-LS-15).  When no projector is
    // configured, the fields remain zero; downstream receivers treat zero as
    // "absent" and fall back to their own coordinate handling.
    float vp_y = 0.0f;
    if (projector_ != nullptr) {
        const auto vp = projector_->project(frame.latitude_rad,
                                            frame.longitude_rad,
                                            frame.height_wgs84_m);
        proto.set_viewer_x_m(vp.x_m);
        proto.set_viewer_y_m(vp.y_m);
        proto.set_viewer_z_m(vp.z_m);
        vp_y = vp.y_m;
    }

    // Diagnostic: print on first 3 broadcasts and whenever AGL is near 0.
    ++broadcast_count_;
    const bool first_few = (broadcast_count_ <= 3);
    const bool agl_near_zero = (frame.agl_m >= 0.f && frame.agl_m < 1.0f);
    const bool agl_crossed = agl_near_zero && !prev_agl_near_zero_;
    prev_agl_near_zero_ = agl_near_zero;
    if (first_few || agl_crossed) {
        std::printf(
            "[broadcast #%u] h_wgs84=%.3f  agl=%.3f  viewer_y=%.3f  "
            "airspeed=%.2f  h_msl=%.3f  %s\n",
            broadcast_count_,
            frame.height_wgs84_m, frame.agl_m, vp_y,
            frame.airspeed_mps, frame.height_msl_m,
            std::isnan(frame.height_wgs84_m) ? "NaN!" : "");
        std::fflush(stdout);
    }

    const std::string serialized = proto.SerializeAsString();

    sockaddr_in dest{};
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(port_);
    dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    sendto(s,
           serialized.data(),
           static_cast<int>(serialized.size()),
           0,
           reinterpret_cast<const sockaddr*>(&dest),
           sizeof(dest));
    // Errors (EAGAIN, EWOULDBLOCK, no listener) are silently ignored —
    // UDP broadcast is fire-and-forget; the sim thread must not block.
}

}  // namespace liteaero::simulation
