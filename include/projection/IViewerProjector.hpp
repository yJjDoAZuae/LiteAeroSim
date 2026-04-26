#pragma once
// IViewerProjector.hpp — Interface for projecting geodetic state into a
// viewer-specific local 3D coordinate system.
//
// The projection module is the architectural boundary between the simulation
// domain (which is viewer-agnostic, using only WGS84 geodetic / NED / body
// frames) and a specific 3D viewer back-end.  Concrete implementations
// implement the axis convention and the world-origin choice for one viewer.
//
// Design authority: docs/architecture/live_sim_view.md §OQ-LS-15 resolution
// and §Implementation Roadmap Task LS-T4.

namespace liteaero::projection {

// ---------------------------------------------------------------------------
// World-origin and projected position value types
// ---------------------------------------------------------------------------

/// Geodetic origin for a viewer's local coordinate system.  All ellipsoidal
/// (WGS84) — the projector is responsible for any datum/curvature math.
struct ViewerOrigin {
    double lat_rad;
    double lon_rad;
    double height_wgs84_m;
};

/// Viewer-local 3D position (axis convention is implementation-defined).
struct ViewerPosition {
    float x_m;
    float y_m;
    float z_m;
};

// ---------------------------------------------------------------------------
// Interface
// ---------------------------------------------------------------------------

class IViewerProjector {
public:
    virtual ~IViewerProjector() = default;

    /// Project a geodetic (WGS84 ellipsoidal) position into the viewer's local
    /// 3D coordinate system, relative to the configured world origin.
    virtual ViewerPosition project(double lat_rad,
                                   double lon_rad,
                                   float  height_wgs84_m) const = 0;
};

}  // namespace liteaero::projection
