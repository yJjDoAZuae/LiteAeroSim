#include "environment/TerrainTile.hpp"
#include <cmath>
#include <stdexcept>

namespace liteaerosim::environment {

// ---------------------------------------------------------------------------
// WGS84 constants (double precision — used for geodetic/ECEF conversions)
// ---------------------------------------------------------------------------
static constexpr double kWgs84A  = 6378137.0;              // semi-major axis (m)
static constexpr double kWgs84E2 = 6.69437999014e-3;       // first eccentricity squared

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Compute the prime-vertical radius of curvature at geodetic latitude lat_rad.
static double primeVerticalRadius(double lat_rad) {
    const double sin_lat = std::sin(lat_rad);
    return kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
}

// Convert geodetic (lat, lon, h) to ECEF (X, Y, Z).
static void geodeticToEcef(double lat_rad, double lon_rad, double height_m,
                            double& X, double& Y, double& Z) {
    const double N       = primeVerticalRadius(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lat = std::sin(lat_rad);
    const double cos_lon = std::cos(lon_rad);
    const double sin_lon = std::sin(lon_rad);
    X = (N + height_m) * cos_lat * cos_lon;
    Y = (N + height_m) * cos_lat * sin_lon;
    Z = (N * (1.0 - kWgs84E2) + height_m) * sin_lat;
}

// Convert ECEF (X, Y, Z) to geodetic (lat, lon, h) using Bowring's iteration.
static void ecefToGeodetic(double X, double Y, double Z,
                            double& lat_rad, double& lon_rad, double& height_m) {
    lon_rad = std::atan2(Y, X);
    const double p = std::sqrt(X * X + Y * Y);
    lat_rad = std::atan2(Z, p * (1.0 - kWgs84E2));  // initial estimate
    for (int i = 0; i < 10; ++i) {
        const double N   = primeVerticalRadius(lat_rad);
        lat_rad = std::atan2(Z + kWgs84E2 * N * std::sin(lat_rad), p);
    }
    const double N = primeVerticalRadius(lat_rad);
    if (std::abs(std::cos(lat_rad)) > 1e-10) {
        height_m = p / std::cos(lat_rad) - N;
    } else {
        height_m = std::abs(Z) / std::sin(lat_rad) - N * (1.0 - kWgs84E2);
    }
}

// Apply the ENU-to-ECEF rotation for centroid at (lat, lon):
//   East  direction in ECEF: [-sinλ,  cosλ,     0    ]
//   North direction in ECEF: [-sinφcosλ, -sinφsinλ,  cosφ]
//   Up    direction in ECEF: [ cosφcosλ,  cosφsinλ,  sinφ]
static void enuToEcefOffset(double lat_rad, double lon_rad,
                             double east_m, double north_m, double up_m,
                             double& dX, double& dY, double& dZ) {
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);
    dX = -sin_lon          * east_m + (-sin_lat * cos_lon) * north_m + (cos_lat * cos_lon) * up_m;
    dY =  cos_lon          * east_m + (-sin_lat * sin_lon) * north_m + (cos_lat * sin_lon) * up_m;
    dZ =  0.0              * east_m +  cos_lat              * north_m +  sin_lat             * up_m;
}

// ---------------------------------------------------------------------------
// TerrainTile
// ---------------------------------------------------------------------------

TerrainTile::TerrainTile(TerrainLod                 lod,
                         GeodeticPoint              centroid,
                         GeodeticAABB               bounds,
                         std::vector<TerrainVertex> vertices,
                         std::vector<TerrainFacet>  facets)
    : lod_(lod),
      centroid_(centroid),
      bounds_(bounds),
      vertices_(std::move(vertices)),
      facets_(std::move(facets)) {}

TerrainLod                        TerrainTile::lod()      const { return lod_; }
const GeodeticPoint&              TerrainTile::centroid() const { return centroid_; }
const GeodeticAABB&               TerrainTile::bounds()   const { return bounds_; }
const std::vector<TerrainVertex>& TerrainTile::vertices() const { return vertices_; }
const std::vector<TerrainFacet>&  TerrainTile::facets()   const { return facets_; }

GeodeticPoint TerrainTile::facetCentroid(std::size_t facet_index) const {
    const TerrainFacet& f = facets_[facet_index];

    // Average ENU offsets of the three vertices (in double for precision).
    const double avg_east  = (static_cast<double>(vertices_[f.v[0]].east_m)  +
                              static_cast<double>(vertices_[f.v[1]].east_m)  +
                              static_cast<double>(vertices_[f.v[2]].east_m))  / 3.0;
    const double avg_north = (static_cast<double>(vertices_[f.v[0]].north_m) +
                              static_cast<double>(vertices_[f.v[1]].north_m) +
                              static_cast<double>(vertices_[f.v[2]].north_m)) / 3.0;
    const double avg_up    = (static_cast<double>(vertices_[f.v[0]].up_m)    +
                              static_cast<double>(vertices_[f.v[1]].up_m)    +
                              static_cast<double>(vertices_[f.v[2]].up_m))    / 3.0;

    // Centroid ECEF.
    double Xc, Yc, Zc;
    geodeticToEcef(centroid_.latitude_rad, centroid_.longitude_rad,
                   centroid_.height_wgs84_m, Xc, Yc, Zc);

    // Apply ENU offset.
    double dX, dY, dZ;
    enuToEcefOffset(centroid_.latitude_rad, centroid_.longitude_rad,
                    avg_east, avg_north, avg_up, dX, dY, dZ);

    // Convert back to geodetic.
    double lat_out, lon_out, h_out;
    ecefToGeodetic(Xc + dX, Yc + dY, Zc + dZ, lat_out, lon_out, h_out);

    return {lat_out, lon_out, static_cast<float>(h_out)};
}

std::array<double, 3> TerrainTile::facetNormal(std::size_t facet_index) const {
    const TerrainFacet& f = facets_[facet_index];

    // Edge vectors in ENU (double precision).
    const double ae = static_cast<double>(vertices_[f.v[1]].east_m)  - vertices_[f.v[0]].east_m;
    const double an = static_cast<double>(vertices_[f.v[1]].north_m) - vertices_[f.v[0]].north_m;
    const double au = static_cast<double>(vertices_[f.v[1]].up_m)    - vertices_[f.v[0]].up_m;
    const double be = static_cast<double>(vertices_[f.v[2]].east_m)  - vertices_[f.v[0]].east_m;
    const double bn = static_cast<double>(vertices_[f.v[2]].north_m) - vertices_[f.v[0]].north_m;
    const double bu = static_cast<double>(vertices_[f.v[2]].up_m)    - vertices_[f.v[0]].up_m;

    // Cross product A × B gives the outward ENU normal (CCW winding from outside).
    double ne = an * bu - au * bn;
    double nn = au * be - ae * bu;
    double nu = ae * bn - an * be;

    // Normalize in ENU.
    const double len = std::sqrt(ne*ne + nn*nn + nu*nu);
    ne /= len;  nn /= len;  nu /= len;

    // Rotate to ECEF.
    double dX, dY, dZ;
    enuToEcefOffset(centroid_.latitude_rad, centroid_.longitude_rad,
                    ne, nn, nu, dX, dY, dZ);
    return {dX, dY, dZ};
}

// ---------------------------------------------------------------------------
// JSON serialization
// ---------------------------------------------------------------------------

nlohmann::json TerrainTile::serializeJson() const {
    nlohmann::json j;
    j["schema_version"] = 1;
    j["lod"]            = static_cast<int>(lod_);
    j["centroid"]       = {
        {"latitude_rad",   centroid_.latitude_rad},
        {"longitude_rad",  centroid_.longitude_rad},
        {"height_wgs84_m", centroid_.height_wgs84_m},
    };
    j["bounds"] = {
        {"lat_min_rad",   bounds_.lat_min_rad},
        {"lat_max_rad",   bounds_.lat_max_rad},
        {"lon_min_rad",   bounds_.lon_min_rad},
        {"lon_max_rad",   bounds_.lon_max_rad},
        {"height_min_m",  bounds_.height_min_m},
        {"height_max_m",  bounds_.height_max_m},
    };
    nlohmann::json jv = nlohmann::json::array();
    for (const auto& v : vertices_) {
        jv.push_back({{"east_m", v.east_m}, {"north_m", v.north_m}, {"up_m", v.up_m}});
    }
    j["vertices"] = std::move(jv);
    nlohmann::json jf = nlohmann::json::array();
    for (const auto& facet : facets_) {
        jf.push_back({
            {"v",     {facet.v[0], facet.v[1], facet.v[2]}},
            {"color", {{"r", facet.color.r}, {"g", facet.color.g}, {"b", facet.color.b}}},
        });
    }
    j["facets"] = std::move(jf);
    return j;
}

TerrainTile TerrainTile::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1) {
        throw std::runtime_error("TerrainTile: unsupported schema_version");
    }
    const TerrainLod lod = static_cast<TerrainLod>(j.at("lod").get<int>());

    const auto& jc = j.at("centroid");
    const GeodeticPoint centroid{
        jc.at("latitude_rad").get<double>(),
        jc.at("longitude_rad").get<double>(),
        jc.at("height_wgs84_m").get<float>(),
    };

    const auto& jb = j.at("bounds");
    const GeodeticAABB bounds{
        jb.at("lat_min_rad").get<double>(),
        jb.at("lat_max_rad").get<double>(),
        jb.at("lon_min_rad").get<double>(),
        jb.at("lon_max_rad").get<double>(),
        jb.at("height_min_m").get<float>(),
        jb.at("height_max_m").get<float>(),
    };

    std::vector<TerrainVertex> vertices;
    vertices.reserve(j.at("vertices").size());
    for (const auto& jv : j.at("vertices")) {
        vertices.push_back({
            jv.at("east_m").get<float>(),
            jv.at("north_m").get<float>(),
            jv.at("up_m").get<float>(),
        });
    }

    std::vector<TerrainFacet> facets;
    facets.reserve(j.at("facets").size());
    for (const auto& jfacet : j.at("facets")) {
        TerrainFacet facet;
        facet.v[0]      = jfacet.at("v")[0].get<uint32_t>();
        facet.v[1]      = jfacet.at("v")[1].get<uint32_t>();
        facet.v[2]      = jfacet.at("v")[2].get<uint32_t>();
        facet.color.r   = jfacet.at("color").at("r").get<uint8_t>();
        facet.color.g   = jfacet.at("color").at("g").get<uint8_t>();
        facet.color.b   = jfacet.at("color").at("b").get<uint8_t>();
        facets.push_back(facet);
    }

    return TerrainTile(lod, centroid, bounds, std::move(vertices), std::move(facets));
}

// ---------------------------------------------------------------------------
// Proto serialization — not yet implemented (Step 11)
// ---------------------------------------------------------------------------

std::vector<uint8_t> TerrainTile::serializeProto() const {
    throw std::runtime_error("TerrainTile::serializeProto: not implemented until Step 11");
}

TerrainTile TerrainTile::deserializeProto(const std::vector<uint8_t>&) {
    throw std::runtime_error("TerrainTile::deserializeProto: not implemented until Step 11");
}

} // namespace liteaerosim::environment
