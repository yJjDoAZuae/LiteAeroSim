#include "environment/Terrain.hpp"
#include <algorithm>

namespace liteaerosim::environment {

float V_Terrain::heightAboveGround_m(float  altitude_m,
                                     double latitude_rad,
                                     double longitude_rad) const {
    return std::max(0.f, altitude_m - elevation_m(latitude_rad, longitude_rad));
}

FlatTerrain::FlatTerrain(float elevation_m) : elevation_m_(elevation_m) {}

float FlatTerrain::elevation_m(double /*latitude_rad*/,
                               double /*longitude_rad*/) const {
    return elevation_m_;
}

} // namespace liteaerosim::environment
