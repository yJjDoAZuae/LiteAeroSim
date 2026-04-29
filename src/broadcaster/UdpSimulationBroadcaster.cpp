#include "broadcaster/UdpSimulationBroadcaster.hpp"
#include "projection/IViewerProjector.hpp"
#include "liteaerosim.pb.h"

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
    if (projector_ != nullptr) {
        const auto vp = projector_->project(frame.latitude_rad,
                                            frame.longitude_rad,
                                            frame.height_wgs84_m);
        proto.set_viewer_x_m(vp.x_m);
        proto.set_viewer_y_m(vp.y_m);
        proto.set_viewer_z_m(vp.z_m);
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
