#pragma once
// Wgs84.hpp — WGS84 geodetic / ECEF / ENU transforms (raw-double API).
//
// Free-function helpers in the liteaero::geodesy namespace.  These are the
// shared primitives consumed by both the simulation domain (TerrainMesh tile
// math) and the viewer-projection module (GodotEnuProjector et al.).
//
// Design authority: docs/architecture/live_sim_view.md §Implementation Roadmap
// Task LS-T1.
//
// Note: liteaero::nav::WGS84 (in liteaero-flight) provides a richer API that
// operates on the GeodeticPosition value type and includes radii, transport
// rates, gravity, qne, etc.  The liteaero::geodesy module is intentionally
// minimal — raw-double parameters, no Eigen — to be cheap to call from
// hot paths (per-frame projection) and to remain dependency-free of Eigen
// at this level.

namespace liteaero::geodesy {

// ---------------------------------------------------------------------------
// WGS84 ellipsoid constants
// ---------------------------------------------------------------------------

inline constexpr double kWgs84A  = 6378137.0;          ///< Semi-major axis, m.
inline constexpr double kWgs84F  = 1.0 / 298.257223563; ///< Flattening.
inline constexpr double kWgs84E2 = 6.69437999014e-3;    ///< First eccentricity squared (precomputed).

// ---------------------------------------------------------------------------
// Radii of curvature
// ---------------------------------------------------------------------------

/// Prime-vertical radius of curvature N(lat), m.
double primeVerticalRadius(double lat_rad);

// ---------------------------------------------------------------------------
// Geodetic <-> ECEF
// ---------------------------------------------------------------------------

/// Convert geodetic (lat, lon, h_WGS84) to ECEF (X, Y, Z).
void geodeticToEcef(double lat_rad, double lon_rad, double h_m,
                    double& X, double& Y, double& Z);

// ---------------------------------------------------------------------------
// ECEF offset <-> ENU at a reference (lat, lon)
// ---------------------------------------------------------------------------

/// Convert an ECEF offset (dX, dY, dZ) into ENU components at the given
/// reference latitude/longitude.  The reference height does not affect the
/// rotation; only lat/lon set the local tangent-plane orientation.
void ecefOffsetToEnu(double ref_lat_rad, double ref_lon_rad,
                     double dX, double dY, double dZ,
                     double& east, double& north, double& up);

/// Convert ENU components (east, north, up) at the given reference into an
/// ECEF offset (dX, dY, dZ).  Inverse of ecefOffsetToEnu.
void enuToEcefOffset(double ref_lat_rad, double ref_lon_rad,
                     double east, double north, double up,
                     double& dX, double& dY, double& dZ);

}  // namespace liteaero::geodesy
