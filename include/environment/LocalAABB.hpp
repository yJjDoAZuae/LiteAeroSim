#pragma once

namespace liteaerosim::environment {

// Primary query interface for simulation-loop terrain queries.
// Specifies a rectangular footprint in the local North-East plane,
// bounded vertically by absolute WGS84 ellipsoidal heights.
struct LocalAABB {
    float half_extent_north_m;  // ±north extent from reference center (m)
    float half_extent_east_m;   // ±east extent from reference center (m)
    float height_min_m;         // WGS84 ellipsoidal height lower bound (m)
    float height_max_m;         // WGS84 ellipsoidal height upper bound (m)
};

} // namespace liteaerosim::environment
