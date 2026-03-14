#pragma once

namespace liteaerosim::environment {

struct TerrainVertex {
    float east_m;   // east displacement from tile centroid in local ENU frame (m)
    float north_m;  // north displacement from tile centroid in local ENU frame (m)
    float up_m;     // vertical displacement from tile centroid, positive up (m)
};

} // namespace liteaerosim::environment
