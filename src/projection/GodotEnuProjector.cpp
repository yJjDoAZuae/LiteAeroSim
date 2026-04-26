// GodotEnuProjector.cpp — Godot/glTF ECEF -> ENU projector.

#include "projection/GodotEnuProjector.hpp"

#include "geodesy/Wgs84.hpp"

namespace liteaero::projection {

GodotEnuProjector::GodotEnuProjector(const ViewerOrigin& origin)
    : origin_(origin)
{
    liteaero::geodesy::geodeticToEcef(
        origin_.lat_rad, origin_.lon_rad, origin_.height_wgs84_m,
        origin_X_, origin_Y_, origin_Z_);
}

ViewerPosition GodotEnuProjector::project(double lat_rad,
                                          double lon_rad,
                                          float  height_wgs84_m) const {
    double X = 0.0, Y = 0.0, Z = 0.0;
    liteaero::geodesy::geodeticToEcef(
        lat_rad, lon_rad, static_cast<double>(height_wgs84_m), X, Y, Z);

    double east = 0.0, north = 0.0, up = 0.0;
    liteaero::geodesy::ecefOffsetToEnu(
        origin_.lat_rad, origin_.lon_rad,
        X - origin_X_, Y - origin_Y_, Z - origin_Z_,
        east, north, up);

    // glTF / Godot axis permutation: X = East, Y = Up, Z = -North.
    return ViewerPosition{
        static_cast<float>(east),
        static_cast<float>(up),
        static_cast<float>(-north),
    };
}

}  // namespace liteaero::projection
