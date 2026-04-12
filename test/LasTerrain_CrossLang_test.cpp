// LasTerrain_CrossLang_test.cpp — cross-language .las_terrain interoperability tests.
//
// Verifies that files written by the Python las_terrain.py writer can be parsed by
// TerrainMesh::deserializeLasTerrain, and that files written by C++ can be parsed
// by the Python read_las_terrain function.
//
// Direction 1 — Python writes, C++ reads:
//   CrossLangTest.ReadsPyFixture reads test/data/las_terrain/py_written.las_terrain
//   (written by Python test_write_py_fixture) and asserts all field values.
//   CTest role: FIXTURES_REQUIRED CrossLangPyFixture.
//
// Direction 2 — C++ writes, Python reads:
//   CrossLangTest.WritesCppFixture writes test/data/las_terrain/cpp_written.las_terrain
//   from a canonical TerrainMesh.  The Python test_read_cpp_fixture reads it.
//   CTest role: FIXTURES_SETUP CrossLangCppFixture.
//
// Canonical fixture values must match the _* constants in
// python/test/test_las_terrain_cross_lang.py exactly.

#include "environment/TerrainMesh.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>

using namespace liteaero::terrain;
using namespace liteaero::simulation;

namespace {

// ---------------------------------------------------------------------------
// Canonical fixture constants.
// MUST match the Python _* constants in test_las_terrain_cross_lang.py.
// ---------------------------------------------------------------------------

constexpr int    kLod             = 3;                    // TerrainLod::L3_MediumDetail
constexpr double kCentroidLatRad  = 0.5235987755982988;   // 30 deg N
constexpr double kCentroidLonRad  = 1.0471975511965976;   // 60 deg E
constexpr float  kCentroidHeightM = 150.0f;
constexpr double kLatMinRad       = 0.52;
constexpr double kLatMaxRad       = 0.53;
constexpr double kLonMinRad       = 1.04;
constexpr double kLonMaxRad       = 1.06;
constexpr float  kHeightMinM      = 100.0f;
constexpr float  kHeightMaxM      = 200.0f;

// ENU vertex data: (east_m, north_m, up_m).
constexpr float kVertices[3][3] = {
    {10.f, 20.f, 30.f},
    {40.f, 50.f, 60.f},
    {70.f, 80.f, 90.f},
};

constexpr uint32_t kFacetIndices[3] = {0, 1, 2};
constexpr uint8_t  kFacetColor[3]   = {200, 100, 50};

const std::filesystem::path kFixtureDir =
    std::filesystem::path(LAS_TEST_DATA_DIR) / "las_terrain";

// ---------------------------------------------------------------------------
// Helper: build the canonical TerrainMesh from the constants above.
// ---------------------------------------------------------------------------

TerrainMesh makeCanonicalMesh()
{
    const GeodeticPoint centroid{kCentroidLatRad, kCentroidLonRad, kCentroidHeightM};
    const GeodeticAABB  bounds{kLatMinRad, kLatMaxRad,
                               kLonMinRad, kLonMaxRad,
                               kHeightMinM, kHeightMaxM};

    std::vector<TerrainVertex> vertices{
        {kVertices[0][0], kVertices[0][1], kVertices[0][2]},
        {kVertices[1][0], kVertices[1][1], kVertices[1][2]},
        {kVertices[2][0], kVertices[2][1], kVertices[2][2]},
    };
    std::vector<TerrainFacet> facets{{
        {kFacetIndices[0], kFacetIndices[1], kFacetIndices[2]},
        {kFacetColor[0],   kFacetColor[1],   kFacetColor[2]},
    }};

    TerrainTile tile(static_cast<TerrainLod>(kLod), centroid, bounds,
                     std::move(vertices), std::move(facets));
    TerrainCell cell;
    cell.addTile(std::move(tile));
    TerrainMesh mesh;
    mesh.addCell(std::move(cell));
    return mesh;
}

// ---------------------------------------------------------------------------
// Helper: assert that tile values match the canonical fixture constants.
// ---------------------------------------------------------------------------

void assertTileMatchesFixture(const TerrainTile& tile)
{
    EXPECT_EQ(static_cast<int>(tile.lod()), kLod);

    EXPECT_NEAR(tile.centroid().latitude_rad,   kCentroidLatRad,  1e-9);
    EXPECT_NEAR(tile.centroid().longitude_rad,  kCentroidLonRad,  1e-9);
    EXPECT_NEAR(tile.centroid().height_wgs84_m, kCentroidHeightM, 1e-3f);

    EXPECT_NEAR(tile.bounds().lat_min_rad,  kLatMinRad,  1e-9);
    EXPECT_NEAR(tile.bounds().lat_max_rad,  kLatMaxRad,  1e-9);
    EXPECT_NEAR(tile.bounds().lon_min_rad,  kLonMinRad,  1e-9);
    EXPECT_NEAR(tile.bounds().lon_max_rad,  kLonMaxRad,  1e-9);
    EXPECT_NEAR(tile.bounds().height_min_m, kHeightMinM, 1e-3f);
    EXPECT_NEAR(tile.bounds().height_max_m, kHeightMaxM, 1e-3f);

    ASSERT_EQ(tile.vertices().size(), 3u);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(tile.vertices()[i].east_m,  kVertices[i][0], 1e-3f) << "vertex " << i << " east_m";
        EXPECT_NEAR(tile.vertices()[i].north_m, kVertices[i][1], 1e-3f) << "vertex " << i << " north_m";
        EXPECT_NEAR(tile.vertices()[i].up_m,    kVertices[i][2], 1e-3f) << "vertex " << i << " up_m";
    }

    ASSERT_EQ(tile.facets().size(), 1u);
    EXPECT_EQ(tile.facets()[0].v[0],    kFacetIndices[0]);
    EXPECT_EQ(tile.facets()[0].v[1],    kFacetIndices[1]);
    EXPECT_EQ(tile.facets()[0].v[2],    kFacetIndices[2]);
    EXPECT_EQ(tile.facets()[0].color.r, kFacetColor[0]);
    EXPECT_EQ(tile.facets()[0].color.g, kFacetColor[1]);
    EXPECT_EQ(tile.facets()[0].color.b, kFacetColor[2]);
}

} // namespace

// ---------------------------------------------------------------------------
// Direction 2 — C++ writes, Python reads.
// CTest role: FIXTURES_SETUP CrossLangCppFixture.
// ---------------------------------------------------------------------------

TEST(CrossLangTest, WritesCppFixture)
{
    std::filesystem::create_directories(kFixtureDir);
    const std::filesystem::path path = kFixtureDir / "cpp_written.las_terrain";

    const std::vector<uint8_t> bytes = makeCanonicalMesh().serializeLasTerrain();

    std::ofstream f(path, std::ios::binary);
    ASSERT_TRUE(f.is_open()) << "Cannot open output path: " << path;
    f.write(reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
    f.close();

    EXPECT_GT(std::filesystem::file_size(path), 0u);
}

// ---------------------------------------------------------------------------
// Direction 1 — Python writes, C++ reads.
// CTest role: FIXTURES_REQUIRED CrossLangPyFixture.
// ---------------------------------------------------------------------------

TEST(CrossLangTest, ReadsPyFixture)
{
    const std::filesystem::path path = kFixtureDir / "py_written.las_terrain";
    if (!std::filesystem::exists(path)) {
        GTEST_SKIP() << "Python fixture not found: " << path
                     << "\nRun CrossLang_PyFixtureSetup via CTest, or: "
                        "uv run pytest python/test/test_las_terrain_cross_lang.py"
                        "::test_write_py_fixture";
    }

    std::ifstream f(path, std::ios::binary);
    ASSERT_TRUE(f.is_open()) << "Cannot open: " << path;
    const std::vector<uint8_t> bytes(
        (std::istreambuf_iterator<char>(f)),
        std::istreambuf_iterator<char>());

    TerrainMesh mesh;
    ASSERT_NO_THROW(mesh.deserializeLasTerrain(bytes))
        << "deserializeLasTerrain threw on Python-written fixture";

    const TerrainCell* cell = mesh.cellAt(kCentroidLatRad, kCentroidLonRad);
    ASSERT_NE(cell, nullptr) << "cellAt returned nullptr — centroid not indexed correctly";

    const TerrainLod lod = static_cast<TerrainLod>(kLod);
    ASSERT_TRUE(cell->hasLod(lod)) << "Expected LOD " << kLod << " not present in cell";

    assertTileMatchesFixture(cell->tile(lod));
}
