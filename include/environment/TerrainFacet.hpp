#pragma once
#include <cstdint>

namespace liteaerosim::environment {

struct FacetColor {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

struct TerrainFacet {
    uint32_t  v[3];    // indices into the tile's vertex array (CCW winding, outward normal)
    FacetColor color;  // packed R8G8B8 representative color
};

} // namespace liteaerosim::environment
