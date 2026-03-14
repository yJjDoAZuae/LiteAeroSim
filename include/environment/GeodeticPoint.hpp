#pragma once

namespace liteaerosim::environment {

struct GeodeticPoint {
    double latitude_rad;    // WGS84 geodetic latitude (rad)
    double longitude_rad;   // WGS84 geodetic longitude (rad)
    float  height_wgs84_m;  // WGS84 ellipsoidal height (m)
};

} // namespace liteaerosim::environment
