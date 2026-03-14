#pragma once
#include "environment/GeodeticPoint.hpp"
#include "environment/GeodeticAABB.hpp"
#include "environment/TerrainVertex.hpp"
#include "environment/TerrainFacet.hpp"
#include <nlohmann/json.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace liteaerosim::environment {

enum class TerrainLod : int {
    L0_Finest         = 0,  // ~10 m vertex spacing
    L1_VeryHighDetail = 1,  // ~30 m
    L2_HighDetail     = 2,  // ~100 m
    L3_MediumDetail   = 3,  // ~300 m
    L4_LowDetail      = 4,  // ~1,000 m
    L5_VeryLowDetail  = 5,  // ~3,000 m
    L6_Coarsest       = 6,  // ~10,000 m
};

class TerrainTile {
public:
    TerrainTile(TerrainLod                 lod,
                GeodeticPoint              centroid,
                GeodeticAABB               bounds,
                std::vector<TerrainVertex> vertices,
                std::vector<TerrainFacet>  facets);

    TerrainLod                          lod()      const;
    const GeodeticPoint&                centroid() const;
    const GeodeticAABB&                 bounds()   const;
    const std::vector<TerrainVertex>&   vertices() const;
    const std::vector<TerrainFacet>&    facets()   const;

    // Returns the geodetic position of facet i's centroid (ENU average, converted to geodetic).
    GeodeticPoint         facetCentroid(std::size_t facet_index) const;

    // Returns the ECEF outward unit normal of facet i.
    std::array<double, 3> facetNormal(std::size_t facet_index) const;

    nlohmann::json       serializeJson()                               const;
    static TerrainTile   deserializeJson(const nlohmann::json&         j);
    std::vector<uint8_t> serializeProto()                              const;
    static TerrainTile   deserializeProto(const std::vector<uint8_t>&  bytes);

private:
    TerrainLod                 lod_;
    GeodeticPoint              centroid_;
    GeodeticAABB               bounds_;
    std::vector<TerrainVertex> vertices_;
    std::vector<TerrainFacet>  facets_;
};

} // namespace liteaerosim::environment
