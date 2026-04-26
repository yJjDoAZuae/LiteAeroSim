"""download.py — Authenticated download from Copernicus Data Space and NASA EarthData.

All network I/O is isolated here; all other terrain pipeline tools consume local files only.

Authentication:
    Copernicus Data Space (DEM, Sentinel-2): OAuth2 client_credentials via
        $COPERNICUS_CLIENT_ID / $COPERNICUS_CLIENT_SECRET environment variables.
    NASA EarthData (SRTM, NASADEM, Landsat, MODIS): .netrc token via
        earthaccess.login().

Download method — Copernicus data:
    Uses the Sentinel Hub Process API (https://sh.dataspace.copernicus.eu/api/v1/process).
    A single POST request returns a GeoTIFF covering exactly the requested bbox.  No STAC
    query or S3 access is required.  The same Bearer token obtained via client_credentials
    is used for both authentication endpoints.

    DEM:     type="dem", demInstance="COPERNICUS_30" (GLO-30, 30 m native; evalscript
             requests FLOAT32 elevation in metres).
    Imagery: type="sentinel-2-l2a"; evalscript returns 4-band UINT16 surface
             reflectance × 10000 in band order [B02, B02, B03, B04] to match the
             synthetic imagery layout expected by colorize.py (bands=[4,3,2]).

Caching:
    Each tile is stored in a persistent per-source cache directory.  If the file already
    exists on disk the download is skipped — no network request is made.

    Default cache root  :  python/cache/terrain/  (project-local, git-ignored)
    Layout              :  <cache_root>/dem/<source>/<bbox_tag>.tif
                           <cache_root>/imagery/<source>/<bbox_tag>.tif

    Override by passing an explicit output_dir.
"""

from __future__ import annotations

import os
from pathlib import Path

BBox = tuple[float, float, float, float]  # (lon_min, lat_min, lon_max, lat_max) degrees

_SH_PROCESS_URL = "https://sh.dataspace.copernicus.eu/api/v1/process"
_TOKEN_URL = (
    "https://identity.dataspace.copernicus.eu"
    "/auth/realms/CDSE/protocol/openid-connect/token"
)

# Evalscript: DEM — FLOAT32 elevation GeoTIFF (metres, orthometric / MSL).
#
# Vertical datum by source (orthometric heights, NOT WGS84 ellipsoidal — these
# values must be passed through geoid_correct.apply_geoid_correction() before
# being treated as ellipsoidal):
#   COPERNICUS_30 / COPERNICUS_90 -> EGM2008
#   NASADEM, SRTM                 -> EGM96
_EVALSCRIPT_DEM = (
    "//VERSION=3\n"
    "function setup(){return{"
    "input:[\"DEM\"],"
    "output:{id:\"default\",bands:1,sampleType:SampleType.FLOAT32}}}\n"
    "function evaluatePixel(s){return[s.DEM]}"
)

# Evalscript: Sentinel-2 L2A — 4-band UINT16 surface reflectance × 10000.
# Band layout matches synthetic imagery and colorize.py sentinel2 config (bands=[4,3,2]):
#   Band 1 — B02 placeholder  (unused)
#   Band 2 — B02 Blue
#   Band 3 — B03 Green
#   Band 4 — B04 Red
_EVALSCRIPT_S2 = (
    "//VERSION=3\n"
    "function setup(){return{"
    "input:[\"B02\",\"B03\",\"B04\"],"
    "output:{id:\"default\",bands:4,sampleType:SampleType.UINT16}}}\n"
    "function evaluatePixel(s){"
    "return[s.B02*10000,s.B02*10000,s.B03*10000,s.B04*10000]}"
)


class DownloadError(IOError):
    """Raised on HTTP errors, authentication failure, or missing coverage."""


def default_cache_dir() -> Path:
    """Return the project-local cache root for downloaded terrain tiles.

    Resolves to python/cache/terrain/ relative to this file's location
    (python/tools/terrain/download.py -> python/cache/terrain/).

    The directory is listed in .gitignore and is never committed.
    """
    return Path(__file__).parent.parent.parent / "cache" / "terrain"


def download_dem(
    bbox_deg: BBox,
    output_dir: Path | None = None,
    source: str = "copernicus_dem_glo30",
) -> list[Path]:
    """Download a DEM GeoTIFF covering bbox_deg via the Sentinel Hub Process API.

    Args:
        bbox_deg:   (lon_min, lat_min, lon_max, lat_max) in degrees.
        output_dir: Directory to store the output.  Defaults to
                    default_cache_dir() / "dem" / source.
        source:     One of "copernicus_dem_glo30", "nasadem", "srtm".

    Returns:
        Single-element list containing the path to the downloaded GeoTIFF.

    Raises:
        DownloadError: on HTTP errors, authentication failure, or unsupported source.
    """
    _SOURCES = {"copernicus_dem_glo30", "nasadem", "srtm"}
    if source not in _SOURCES:
        raise DownloadError(f"Unsupported DEM source '{source}'. Choose from: {_SOURCES}")

    if output_dir is None:
        output_dir = default_cache_dir() / "dem" / source
    output_dir.mkdir(parents=True, exist_ok=True)

    if source == "copernicus_dem_glo30":
        return _sh_download_dem(bbox_deg, output_dir)
    else:
        return _download_earthdata_dem(bbox_deg, output_dir, source)


def download_imagery(
    bbox_deg: BBox,
    output_dir: Path | None = None,
    source: str = "sentinel2",
    lod: int = 0,
    time_from: str = "2023-06-01T00:00:00Z",
    time_to: str = "2025-09-30T00:00:00Z",
) -> list[Path]:
    """Download a cloud-free imagery GeoTIFF covering bbox_deg.

    Args:
        bbox_deg:   (lon_min, lat_min, lon_max, lat_max) in degrees.
        output_dir: Directory to store the output.  Defaults to
                    default_cache_dir() / "imagery" / source.
        source:     One of "sentinel2", "landsat9", "modis".
        lod:        Requested LOD (used to select appropriate resolution).
        time_from:  Start of the time window for scene selection (ISO 8601).
        time_to:    End of the time window for scene selection (ISO 8601).

    Returns:
        Single-element list containing the path to the downloaded GeoTIFF.

    Raises:
        DownloadError: on HTTP errors, authentication failure, or unsupported source.
    """
    _SOURCES = {"sentinel2", "landsat9", "modis"}
    if source not in _SOURCES:
        raise DownloadError(f"Unsupported imagery source '{source}'. Choose from: {_SOURCES}")

    if output_dir is None:
        output_dir = default_cache_dir() / "imagery" / source
    output_dir.mkdir(parents=True, exist_ok=True)

    if source == "sentinel2":
        return _sh_download_imagery(bbox_deg, output_dir, time_from, time_to)
    else:
        return _download_earthdata_imagery(bbox_deg, output_dir, source, lod)


# ---------------------------------------------------------------------------
# Sentinel Hub Process API helpers
# ---------------------------------------------------------------------------

def _copernicus_token() -> str:
    """Obtain an OAuth2 Bearer token from Copernicus Data Space (client_credentials)."""
    import requests

    client_id = os.environ.get("COPERNICUS_CLIENT_ID")
    client_secret = os.environ.get("COPERNICUS_CLIENT_SECRET")
    if not client_id or not client_secret:
        raise DownloadError(
            "Copernicus authentication requires COPERNICUS_CLIENT_ID and "
            "COPERNICUS_CLIENT_SECRET environment variables."
        )
    resp = requests.post(
        _TOKEN_URL,
        data={
            "grant_type": "client_credentials",
            "client_id": client_id,
            "client_secret": client_secret,
        },
        timeout=30,
    )
    if not resp.ok:
        raise DownloadError(
            f"Copernicus token request failed: {resp.status_code} {resp.text[:200]}"
        )
    return str(resp.json()["access_token"])


def _bbox_tag(bbox_deg: BBox) -> str:
    """Stable filename fragment derived from bbox coordinates."""
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    ns = lambda v, p, n: f"{abs(v):.5f}{p if v >= 0 else n}"
    return (
        f"{ns(lat_min,'N','S')}_{ns(lat_max,'N','S')}"
        f"_{ns(lon_min,'E','W')}_{ns(lon_max,'E','W')}"
    )


def _sh_process(token: str, body: dict) -> bytes:
    """POST a Sentinel Hub Process API request and return raw GeoTIFF bytes."""
    import requests

    resp = requests.post(
        _SH_PROCESS_URL,
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
        },
        json=body,
        timeout=300,
    )
    if resp.status_code == 429:
        raise DownloadError(
            "Sentinel Hub rate limit exceeded. "
            f"Retry-After: {resp.headers.get('Retry-After', '?')} ms"
        )
    if not resp.ok:
        raise DownloadError(
            f"Sentinel Hub Process API request failed: "
            f"{resp.status_code} {resp.text[:400]}"
        )
    return resp.content


def _sh_download_dem(bbox_deg: BBox, output_dir: Path) -> list[Path]:
    """Download Copernicus DEM GLO-30 via Sentinel Hub Process API.

    Returns a single FLOAT32 GeoTIFF in EPSG:4326 with elevation in metres.
    Resolution is 0.000090 deg/pixel (≈ 10 m at mid-latitudes) — the LOD 0 grid
    spacing used by triangulate.py.  The GLO-30 native resolution is 0.000278 deg
    (≈ 30 m); the Process API bilinearly upsamples to the requested resolution.
    """
    fname = output_dir / f"dem_copernicus30_{_bbox_tag(bbox_deg)}.tif"
    if fname.exists():
        return [fname]

    lon_min, lat_min, lon_max, lat_max = bbox_deg
    token = _copernicus_token()

    body = {
        "input": {
            "bounds": {
                "bbox": [lon_min, lat_min, lon_max, lat_max],
                "properties": {"crs": "http://www.opengis.net/def/crs/EPSG/0/4326"},
            },
            "data": [
                {
                    "type": "dem",
                    "dataFilter": {"demInstance": "COPERNICUS_30"},
                    "processing": {"upsampling": "BILINEAR"},
                }
            ],
        },
        "output": {
            "resx": 0.000090,
            "resy": 0.000090,
            "responses": [
                {"identifier": "default", "format": {"type": "image/tiff"}}
            ],
        },
        "evalscript": _EVALSCRIPT_DEM,
    }

    tif_bytes = _sh_process(token, body)
    fname.write_bytes(tif_bytes)
    return [fname]


def _sh_download_imagery(
    bbox_deg: BBox,
    output_dir: Path,
    time_from: str,
    time_to: str,
) -> list[Path]:
    """Download Sentinel-2 L2A imagery via Sentinel Hub Process API.

    Returns a 4-band UINT16 GeoTIFF in EPSG:4326 at 0.000090 deg/pixel (≈ 10 m).
    Band layout: [B02 placeholder, B02 Blue, B03 Green, B04 Red] — matches
    colorize.py sentinel2 config (bands=[4,3,2]).
    The Process API applies mosaickingOrder=leastCC to automatically select
    the least cloudy scene within the requested time window.
    """
    tag = f"{_bbox_tag(bbox_deg)}_{time_from[:10]}_{time_to[:10]}"
    fname = output_dir / f"s2l2a_{tag}.tif"
    if fname.exists():
        return [fname]

    lon_min, lat_min, lon_max, lat_max = bbox_deg
    token = _copernicus_token()

    body = {
        "input": {
            "bounds": {
                "bbox": [lon_min, lat_min, lon_max, lat_max],
                "properties": {"crs": "http://www.opengis.net/def/crs/EPSG/0/4326"},
            },
            "data": [
                {
                    "type": "sentinel-2-l2a",
                    "dataFilter": {
                        "timeRange": {"from": time_from, "to": time_to},
                        "maxCloudCoverage": 30,
                        "mosaickingOrder": "leastCC",
                    },
                    "processing": {
                        "upsampling": "BILINEAR",
                        "downsampling": "BILINEAR",
                        "harmonizeValues": True,
                    },
                }
            ],
        },
        "output": {
            "resx": 0.000090,
            "resy": 0.000090,
            "responses": [
                {"identifier": "default", "format": {"type": "image/tiff"}}
            ],
        },
        "evalscript": _EVALSCRIPT_S2,
    }

    tif_bytes = _sh_process(token, body)
    fname.write_bytes(tif_bytes)
    return [fname]


# ---------------------------------------------------------------------------
# NASA EarthData helpers
# ---------------------------------------------------------------------------

def _download_earthdata_dem(bbox_deg: BBox, output_dir: Path, source: str) -> list[Path]:
    """Download SRTM / NASADEM tiles via earthaccess."""
    import earthaccess

    earthaccess.login(strategy="netrc")
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    collection_id = "SRTMGL1" if source == "srtm" else "NASADEM_HGT"

    results = earthaccess.search_data(
        short_name=collection_id,
        bounding_box=(lon_min, lat_min, lon_max, lat_max),
    )
    if not results:
        return []
    return [Path(p) for p in earthaccess.download(results, str(output_dir))]


def _download_earthdata_imagery(
    bbox_deg: BBox, output_dir: Path, source: str, lod: int
) -> list[Path]:
    """Download Landsat-9 / MODIS imagery tiles via earthaccess."""
    import earthaccess

    earthaccess.login(strategy="netrc")
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    collection_ids = {"landsat9": "HLSL30", "modis": "MCD43A4"}
    short_name = collection_ids[source]

    results = earthaccess.search_data(
        short_name=short_name,
        bounding_box=(lon_min, lat_min, lon_max, lat_max),
    )
    if not results:
        return []
    return [Path(p) for p in earthaccess.download(results, str(output_dir))]
