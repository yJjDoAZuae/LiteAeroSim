// Wgs84.cpp — WGS84 geodetic / ECEF / ENU transforms (raw-double API).
//
// Implementation of liteaero::geodesy free functions declared in
// include/geodesy/Wgs84.hpp.  Lifted verbatim from the static helpers
// previously embedded in src/environment/TerrainMesh.cpp (LS-T1).

#include "geodesy/Wgs84.hpp"

#include <cmath>

namespace liteaero::geodesy {

double primeVerticalRadius(double lat_rad) {
    const double s = std::sin(lat_rad);
    return kWgs84A / std::sqrt(1.0 - kWgs84E2 * s * s);
}

void geodeticToEcef(double lat_rad, double lon_rad, double h_m,
                    double& X, double& Y, double& Z) {
    const double N  = primeVerticalRadius(lat_rad);
    const double cl = std::cos(lat_rad), sl = std::sin(lat_rad);
    const double co = std::cos(lon_rad), so = std::sin(lon_rad);
    X = (N + h_m) * cl * co;
    Y = (N + h_m) * cl * so;
    Z = (N * (1.0 - kWgs84E2) + h_m) * sl;
}

void ecefOffsetToEnu(double ref_lat_rad, double ref_lon_rad,
                     double dX, double dY, double dZ,
                     double& east, double& north, double& up) {
    const double sl = std::sin(ref_lat_rad), cl = std::cos(ref_lat_rad);
    const double so = std::sin(ref_lon_rad), co = std::cos(ref_lon_rad);
    east  = -so * dX           + co * dY;
    north = -sl * co * dX      - sl * so * dY      + cl * dZ;
    up    =  cl * co * dX      + cl * so * dY      + sl * dZ;
}

void enuToEcefOffset(double ref_lat_rad, double ref_lon_rad,
                     double east, double north, double up,
                     double& dX, double& dY, double& dZ) {
    const double sl = std::sin(ref_lat_rad), cl = std::cos(ref_lat_rad);
    const double so = std::sin(ref_lon_rad), co = std::cos(ref_lon_rad);
    dX = -so * east  + (-sl * co) * north  + (cl * co) * up;
    dY =  co * east  + (-sl * so) * north  + (cl * so) * up;
    dZ =                cl * north         +  sl * up;
}

}  // namespace liteaero::geodesy
