"""generate_egm_grid.py — Build the C++ runtime EGM2008 grid binary from PROJ.

Queries pyproj's EGM2008 transformer over a regular lat/lon grid and writes the
result in the binary format consumed by `liteaero::geodesy::Egm2008Geoid`
(see include/geodesy/Egm2008Geoid.hpp).

This tool runs once per machine: PROJ caches the underlying datum-shift grid in
its user-writable directory after the first network access; subsequent runs are
offline.

Default coverage: global, 0.25° spacing (1440 x 720 = ~4 MB binary).
Worst-case bilinear interpolation error against the underlying PROJ grid is
sub-meter, well below the residual atmospheric / instrumentation tolerance the
simulation cares about.

Output path (default): liteaero-sim/data/proj_grids/egm2008.bin
"""

from __future__ import annotations

import argparse
import logging
import struct
from pathlib import Path

import numpy as np
import pyproj.network
from pyproj import CRS, Transformer

_log = logging.getLogger("generate_egm_grid")

# Compound CRS: WGS84 + EGM2008 height -> WGS84 ellipsoidal height.
_EGM2008_CRS = CRS.from_epsg(9518)
_WGS84_3D = CRS.from_epsg(4979)


_MAGIC = b"EGM2008\x00"
_FORMAT_VERSION = 1


def generate_egm_grid(
    output_path: Path,
    *,
    lat_min_deg: float = -90.0,
    lat_max_deg: float = +90.0,
    lon_min_deg: float = -180.0,
    lon_max_deg: float = +180.0,
    spacing_deg: float = 0.25,
) -> Path:
    """Write a binary EGM2008 grid file at the given path.

    Format (little-endian):
        [8  bytes] magic = "EGM2008\\0"
        [4  bytes] format_version (uint32) = 1
        [8  bytes] lat_min_deg (float64)
        [8  bytes] lat_max_deg (float64)
        [8  bytes] lon_min_deg (float64)
        [8  bytes] lon_max_deg (float64)
        [4  bytes] n_lat (uint32)
        [4  bytes] n_lon (uint32)
        [N  bytes] grid (float32, row-major: lat-major outer, lon-major inner)

    Returns the absolute path to the written file.
    """
    pyproj.network.set_network_enabled(True)

    n_lat = int(round((lat_max_deg - lat_min_deg) / spacing_deg)) + 1
    n_lon = int(round((lon_max_deg - lon_min_deg) / spacing_deg)) + 1

    lats = np.linspace(lat_min_deg, lat_max_deg, n_lat)
    lons = np.linspace(lon_min_deg, lon_max_deg, n_lon)

    transformer = Transformer.from_crs(_EGM2008_CRS, _WGS84_3D, always_xy=True)

    _log.info(
        "Sampling EGM2008 over %.1f..%.1f lat, %.1f..%.1f lon at %.4f deg "
        "(%d x %d points)",
        lat_min_deg, lat_max_deg, lon_min_deg, lon_max_deg,
        spacing_deg, n_lat, n_lon,
    )

    grid = np.zeros((n_lat, n_lon), dtype=np.float32)
    for i, lat in enumerate(lats):
        lon_row = lons
        lat_row = np.full_like(lon_row, lat)
        h_in = np.zeros_like(lon_row)
        # Transformer maps (lon, lat, h_orthometric) -> (lon, lat, h_ellipsoidal).
        # At h_orthometric = 0 the output ellipsoidal height equals N directly.
        _, _, h_ellipsoidal = transformer.transform(lon_row, lat_row, h_in)
        grid[i, :] = h_ellipsoidal.astype(np.float32)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "wb") as f:
        f.write(_MAGIC)
        f.write(struct.pack("<I", _FORMAT_VERSION))
        f.write(struct.pack("<dddd",
                            lat_min_deg, lat_max_deg,
                            lon_min_deg, lon_max_deg))
        f.write(struct.pack("<II", n_lat, n_lon))
        f.write(grid.tobytes(order="C"))

    _log.info("Wrote %s (%.1f MB)", output_path,
              output_path.stat().st_size / (1024 * 1024))
    return output_path


def default_output_path() -> Path:
    """Return the canonical runtime path: liteaero-sim/data/proj_grids/egm2008.bin."""
    project_root = Path(__file__).resolve().parents[3]
    return project_root / "data" / "proj_grids" / "egm2008.bin"


def _main() -> None:
    parser = argparse.ArgumentParser(
        prog="generate_egm_grid",
        description="Build the C++ runtime EGM2008 geoid grid binary from PROJ.",
    )
    parser.add_argument(
        "--output", type=Path, default=None,
        help="Output binary file path (default: data/proj_grids/egm2008.bin).",
    )
    parser.add_argument(
        "--spacing-deg", type=float, default=0.25,
        help="Grid spacing in degrees (default 0.25).",
    )
    parser.add_argument("--lat-min", type=float, default=-90.0)
    parser.add_argument("--lat-max", type=float, default=+90.0)
    parser.add_argument("--lon-min", type=float, default=-180.0)
    parser.add_argument("--lon-max", type=float, default=+180.0)
    parser.add_argument("--force", action="store_true",
                        help="Regenerate even if the output file already exists.")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s  %(levelname)-7s  %(message)s",
                        datefmt="%H:%M:%S")

    out = args.output or default_output_path()
    if out.exists() and not args.force:
        print(f"egm2008 grid already present: {out}")
        return

    generate_egm_grid(
        out,
        lat_min_deg=args.lat_min, lat_max_deg=args.lat_max,
        lon_min_deg=args.lon_min, lon_max_deg=args.lon_max,
        spacing_deg=args.spacing_deg,
    )
    print(f"egm2008 grid ready -> {out}")


if __name__ == "__main__":
    _main()
