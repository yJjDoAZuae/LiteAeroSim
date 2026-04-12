"""Cross-language interoperability tests for the .las_terrain binary format.

Verifies that files written by Python (write_las_terrain) can be read by C++
(TerrainMesh::deserializeLasTerrain), and that files written by C++ can be read
by Python (read_las_terrain).

Direction 1 — Python writes, C++ reads:
    test_write_py_fixture  →  CrossLang_PyFixtureRead (CTest, runs liteaerosim_test)

Direction 2 — C++ writes, Python reads:
    CrossLang_CppFixtureSetup (CTest, runs liteaerosim_test)  →  test_read_cpp_fixture

CTest fixture ordering is declared in test/CMakeLists.txt.  Both tests can also be
run directly with pytest for development, with the caveat that test_read_cpp_fixture
will skip if the C++ fixture has not been written yet.

Canonical fixture values must match the constants in
test/LasTerrain_CrossLang_test.cpp exactly.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from las_terrain import TerrainTileData, read_las_terrain, write_las_terrain

# ---------------------------------------------------------------------------
# Canonical fixture values.
# MUST match the k* constants in test/LasTerrain_CrossLang_test.cpp.
# ---------------------------------------------------------------------------

# Absolute path to the shared fixture directory.
_FIXTURE_DIR = Path(__file__).resolve().parent.parent.parent / "test" / "data" / "las_terrain"

_LOD = 3                             # TerrainLod::L3_MediumDetail
_CENTROID_LAT_RAD = 0.5235987755982988   # 30 deg N
_CENTROID_LON_RAD = 1.0471975511965976   # 60 deg E
_CENTROID_HEIGHT_M = 150.0
_LAT_MIN_RAD  = 0.52
_LAT_MAX_RAD  = 0.53
_LON_MIN_RAD  = 1.04
_LON_MAX_RAD  = 1.06
_HEIGHT_MIN_M = 100.0
_HEIGHT_MAX_M = 200.0

_VERTICES = np.array([               # ENU (east_m, north_m, up_m)
    [10.0, 20.0, 30.0],
    [40.0, 50.0, 60.0],
    [70.0, 80.0, 90.0],
], dtype=np.float32)

_INDICES = np.array([[0, 1, 2]], dtype=np.uint32)
_COLORS  = np.array([[200, 100, 50]], dtype=np.uint8)


def _make_fixture_tile() -> TerrainTileData:
    return TerrainTileData(
        lod=_LOD,
        centroid_lat_rad=_CENTROID_LAT_RAD,
        centroid_lon_rad=_CENTROID_LON_RAD,
        centroid_height_m=_CENTROID_HEIGHT_M,
        lat_min_rad=_LAT_MIN_RAD,
        lat_max_rad=_LAT_MAX_RAD,
        lon_min_rad=_LON_MIN_RAD,
        lon_max_rad=_LON_MAX_RAD,
        height_min_m=_HEIGHT_MIN_M,
        height_max_m=_HEIGHT_MAX_M,
        vertices=_VERTICES.copy(),
        indices=_INDICES.copy(),
        colors=_COLORS.copy(),
    )


# ---------------------------------------------------------------------------
# Direction 1 — Python writes, C++ reads.
# CTest role: FIXTURES_SETUP CrossLangPyFixture.
# The C++ counterpart is CrossLangTest.ReadsPyFixture in LasTerrain_CrossLang_test.cpp.
# ---------------------------------------------------------------------------

def test_write_py_fixture() -> None:
    """Write the canonical fixture so the C++ reader test can verify it."""
    _FIXTURE_DIR.mkdir(parents=True, exist_ok=True)
    path = _FIXTURE_DIR / "py_written.las_terrain"
    write_las_terrain(path, [_make_fixture_tile()])
    assert path.exists()
    assert path.stat().st_size > 0


# ---------------------------------------------------------------------------
# Direction 2 — C++ writes, Python reads.
# CTest role: FIXTURES_REQUIRED CrossLangCppFixture.
# The C++ counterpart is CrossLangTest.WritesCppFixture in LasTerrain_CrossLang_test.cpp.
# ---------------------------------------------------------------------------

def test_read_cpp_fixture() -> None:
    """Read the fixture written by C++ and verify all field values match the canonical spec."""
    path = _FIXTURE_DIR / "cpp_written.las_terrain"
    if not path.exists():
        pytest.skip(
            f"C++ fixture not found at {path}. "
            "Run CrossLang_CppFixtureSetup via CTest, or: "
            "build/liteaerosim_test --gtest_filter=CrossLangTest.WritesCppFixture"
        )

    tiles = read_las_terrain(path)
    assert len(tiles) == 1
    t = tiles[0]

    assert t.lod == _LOD

    assert abs(t.centroid_lat_rad  - _CENTROID_LAT_RAD)  < 1e-9
    assert abs(t.centroid_lon_rad  - _CENTROID_LON_RAD)  < 1e-9
    assert abs(t.centroid_height_m - _CENTROID_HEIGHT_M) < 1e-3

    assert abs(t.lat_min_rad  - _LAT_MIN_RAD)  < 1e-9
    assert abs(t.lat_max_rad  - _LAT_MAX_RAD)  < 1e-9
    assert abs(t.lon_min_rad  - _LON_MIN_RAD)  < 1e-9
    assert abs(t.lon_max_rad  - _LON_MAX_RAD)  < 1e-9
    assert abs(t.height_min_m - _HEIGHT_MIN_M) < 1e-3
    assert abs(t.height_max_m - _HEIGHT_MAX_M) < 1e-3

    assert t.vertices.shape == (3, 3)
    np.testing.assert_array_almost_equal(t.vertices, _VERTICES, decimal=3)
    np.testing.assert_array_equal(t.indices, _INDICES)
    np.testing.assert_array_equal(t.colors,  _COLORS)
