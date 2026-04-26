// Egm2008Geoid.cpp — EGM2008 geoid undulation lookup implementation.

#include "geodesy/Egm2008Geoid.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace liteaero::geodesy {

namespace {

constexpr char kMagic[8] = {'E', 'G', 'M', '2', '0', '0', '8', '\0'};
constexpr std::uint32_t kFormatVersion = 1;

// Read a little-endian POD from the stream; throws on EOF.
template <typename T>
T readLE(std::ifstream& f, const char* what) {
    T value{};
    f.read(reinterpret_cast<char*>(&value), sizeof(T));
    if (!f) {
        throw std::runtime_error(
            std::string("Egm2008Geoid: failed to read ") + what);
    }
    return value;
}

}  // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Egm2008Geoid::Egm2008Geoid(const std::filesystem::path& grid_file) {
    std::ifstream f(grid_file, std::ios::binary);
    if (!f) {
        throw std::runtime_error(
            "Egm2008Geoid: cannot open grid file: " + grid_file.string());
    }

    char magic[8] = {};
    f.read(magic, sizeof(magic));
    if (!f || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
        throw std::runtime_error(
            "Egm2008Geoid: bad magic in " + grid_file.string());
    }

    const auto version = readLE<std::uint32_t>(f, "format_version");
    if (version != kFormatVersion) {
        throw std::runtime_error(
            "Egm2008Geoid: unsupported format_version "
          + std::to_string(version));
    }

    lat_min_deg_ = readLE<double>(f, "lat_min_deg");
    lat_max_deg_ = readLE<double>(f, "lat_max_deg");
    lon_min_deg_ = readLE<double>(f, "lon_min_deg");
    lon_max_deg_ = readLE<double>(f, "lon_max_deg");
    n_lat_       = readLE<std::uint32_t>(f, "n_lat");
    n_lon_       = readLE<std::uint32_t>(f, "n_lon");

    if (n_lat_ < 2 || n_lon_ < 2) {
        throw std::runtime_error(
            "Egm2008Geoid: grid must be at least 2x2");
    }
    if (lat_max_deg_ <= lat_min_deg_ || lon_max_deg_ <= lon_min_deg_) {
        throw std::runtime_error(
            "Egm2008Geoid: invalid grid extent");
    }

    grid_.resize(static_cast<std::size_t>(n_lat_) * n_lon_);
    f.read(reinterpret_cast<char*>(grid_.data()),
           static_cast<std::streamsize>(grid_.size() * sizeof(float)));
    if (!f) {
        throw std::runtime_error(
            "Egm2008Geoid: short grid payload in " + grid_file.string());
    }
}

// ---------------------------------------------------------------------------
// Bilinear interpolation
// ---------------------------------------------------------------------------

float Egm2008Geoid::undulation_m(double lat_rad, double lon_rad) const {
    constexpr double kRadToDeg = 57.29577951308232;  // 180/pi
    const double lat_deg = lat_rad * kRadToDeg;
    const double lon_deg = lon_rad * kRadToDeg;

    // Clamp to grid extent.  The geoid is smooth and the canonical PROJ grid
    // has global coverage, so clamping at the boundary is safe.
    const double lat_clamped = std::clamp(lat_deg, lat_min_deg_, lat_max_deg_);
    const double lon_clamped = std::clamp(lon_deg, lon_min_deg_, lon_max_deg_);

    const double dlat = (lat_max_deg_ - lat_min_deg_) / (n_lat_ - 1);
    const double dlon = (lon_max_deg_ - lon_min_deg_) / (n_lon_ - 1);

    const double u = (lat_clamped - lat_min_deg_) / dlat;  // [0, n_lat-1]
    const double v = (lon_clamped - lon_min_deg_) / dlon;  // [0, n_lon-1]

    const auto i0 = static_cast<std::uint32_t>(std::floor(u));
    const auto j0 = static_cast<std::uint32_t>(std::floor(v));
    const auto i1 = std::min(i0 + 1u, n_lat_ - 1u);
    const auto j1 = std::min(j0 + 1u, n_lon_ - 1u);

    const float fu = static_cast<float>(u - i0);
    const float fv = static_cast<float>(v - j0);

    const float n00 = grid_[i0 * n_lon_ + j0];
    const float n01 = grid_[i0 * n_lon_ + j1];
    const float n10 = grid_[i1 * n_lon_ + j0];
    const float n11 = grid_[i1 * n_lon_ + j1];

    const float n0 = n00 * (1.f - fv) + n01 * fv;  // interp along lon at lat=i0
    const float n1 = n10 * (1.f - fv) + n11 * fv;  // interp along lon at lat=i1
    return n0 * (1.f - fu) + n1 * fu;              // interp along lat
}

// ---------------------------------------------------------------------------
// MSL <-> ellipsoidal conversions
// ---------------------------------------------------------------------------

float Egm2008Geoid::wgs84ToMsl_m(double lat_rad, double lon_rad,
                                  float h_wgs84_m) const {
    return h_wgs84_m - undulation_m(lat_rad, lon_rad);
}

float Egm2008Geoid::mslToWgs84_m(double lat_rad, double lon_rad,
                                  float h_msl_m) const {
    return h_msl_m + undulation_m(lat_rad, lon_rad);
}

}  // namespace liteaero::geodesy
