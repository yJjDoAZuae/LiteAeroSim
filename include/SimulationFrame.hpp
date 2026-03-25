#pragma once

namespace liteaero::simulation {

// Value object representing one timestep of live simulation state.
// Populated by the Application Layer and passed to an ISimulationBroadcaster
// (Interface Layer) for transport to the renderer.  No I/O here.
struct SimulationFrame {
    double timestamp_s;         // simulation time (s)
    double latitude_rad;        // vehicle geodetic latitude, WGS84 (rad)
    double longitude_rad;       // vehicle geodetic longitude, WGS84 (rad)
    float  height_wgs84_m;      // vehicle ellipsoidal altitude (m)
    float  q_w;                 // body-to-NED attitude quaternion
    float  q_x;
    float  q_y;
    float  q_z;
    float  velocity_north_mps;
    float  velocity_east_mps;
    float  velocity_down_mps;
};

} // namespace liteaero::simulation
