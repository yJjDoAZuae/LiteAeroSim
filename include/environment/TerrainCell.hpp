#pragma once
#include "environment/GeodeticAABB.hpp"
#include "environment/TerrainTile.hpp"
#include <array>
#include <optional>
#include <stdexcept>

namespace liteaerosim::environment {

class TerrainCell {
public:
    void               addTile(TerrainTile tile);
    bool               hasLod(TerrainLod lod)    const;
    const TerrainTile& tile(TerrainLod lod)       const;  // throws std::out_of_range if absent
    TerrainLod         finestAvailableLod()       const;
    TerrainLod         coarsestAvailableLod()     const;

    // Geographic bounds of the cell (union of all tile bounds — identical per cell).
    const GeodeticAABB& bounds() const;

private:
    static constexpr int kLodCount = 7;
    std::array<std::optional<TerrainTile>, kLodCount> tiles_;  // indexed by TerrainLod
};

} // namespace liteaerosim::environment
