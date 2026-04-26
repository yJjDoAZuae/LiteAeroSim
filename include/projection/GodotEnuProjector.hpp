#pragma once
// GodotEnuProjector.hpp — IViewerProjector implementation for the Godot 4 /
// glTF 2.0 axis convention.
//
// Axis convention (from live_sim_view.md §Coordinate System):
//   Godot/glTF X = ENU East
//   Godot/glTF Y = ENU Up
//   Godot/glTF Z = -ENU North
//
// The projector performs full curvature-aware ECEF -> ENU at the configured
// world origin (no flat-Earth approximation), matching the geometry used by
// the terrain build pipeline (export_gltf.py).  This is the resolution to
// Issue 7 in live_sim_view.md.

#include "projection/IViewerProjector.hpp"

namespace liteaero::projection {

class GodotEnuProjector : public IViewerProjector {
public:
    explicit GodotEnuProjector(const ViewerOrigin& origin);

    ViewerPosition project(double lat_rad,
                           double lon_rad,
                           float  height_wgs84_m) const override;

private:
    ViewerOrigin origin_;

    // Cached origin ECEF (computed once at construction; reused per project()).
    double origin_X_ = 0.0;
    double origin_Y_ = 0.0;
    double origin_Z_ = 0.0;
};

}  // namespace liteaero::projection
