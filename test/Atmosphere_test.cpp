#include "environment/Atmosphere.hpp"
#include "geodesy/Egm2008Geoid.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float kTol001pct = 1e-4f;   // 0.01%
static constexpr float kTol01pct  = 1e-3f;   // 0.1%

// ---------------------------------------------------------------------------
// T1: ISA SL values match ICAO Doc 7488 to 0.01%
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, SeaLevelISA_MatchesICAOTableValues) {
    Atmosphere atm;
    AtmosphericState s = atm.state(0.f);

    EXPECT_NEAR(s.temperature_k,      288.15f,  288.15f  * kTol001pct);
    EXPECT_NEAR(s.pressure_pa,        101325.f, 101325.f * kTol001pct);
    EXPECT_NEAR(s.density_kgm3,       1.225f,   1.225f   * kTol001pct);
    EXPECT_NEAR(s.speed_of_sound_mps, 340.29f,  340.29f  * kTol001pct);
    EXPECT_NEAR(s.density_altitude_m, 0.f,      1.f);  // within 1 m of 0
}

// ---------------------------------------------------------------------------
// T2: Tropopause temperature at 11 000 m
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, Tropopause_TemperatureMatches) {
    Atmosphere atm;
    AtmosphericState s = atm.state(11000.f);
    EXPECT_NEAR(s.temperature_k, 216.65f, 216.65f * kTol01pct);
}

// ---------------------------------------------------------------------------
// T3: density_ratio at 0 m is exactly 1.0
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, DensityRatioAtSeaLevel_IsOne) {
    Atmosphere atm;
    EXPECT_FLOAT_EQ(atm.density_ratio(0.f), 1.f);
}

// ---------------------------------------------------------------------------
// T4: density_ratio at 5 000 m vs ICAO standard (σ ≈ 0.6012)
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, DensityRatioAt5000m_MatchesICAOTable) {
    Atmosphere atm;
    // ICAO standard density ratio at 5000 m geopotential ≈ 0.60117
    EXPECT_NEAR(atm.density_ratio(5000.f), 0.60117f, 0.60117f * kTol01pct);
}

// ---------------------------------------------------------------------------
// T5: density strictly decreasing from 0 to 20 000 m
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, DensityMonotonicallyDecreasing) {
    Atmosphere atm;
    float prev = atm.state(0.f).density_kgm3;
    for (int h = 500; h <= 20000; h += 500) {
        float curr = atm.state(static_cast<float>(h)).density_kgm3;
        EXPECT_LT(curr, prev) << "Not decreasing at h=" << h;
        prev = curr;
    }
}

// ---------------------------------------------------------------------------
// T6: ISA+20 — temperature shifted, pressure unchanged, density lower
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, NonStandardDay_TemperatureShiftedPressureUnchanged) {
    Atmosphere atm_isa;
    Atmosphere atm_warm({.delta_temperature_k = 20.f});

    AtmosphericState s_isa  = atm_isa.state(0.f);
    AtmosphericState s_warm = atm_warm.state(0.f);

    EXPECT_NEAR(s_warm.temperature_k, 308.15f, 308.15f * kTol001pct);
    // Pressure uses ISA temperature only — must equal ISA SL pressure
    EXPECT_NEAR(s_warm.pressure_pa, s_isa.pressure_pa, s_isa.pressure_pa * kTol001pct);
    // Warmer → less dense
    EXPECT_LT(s_warm.density_kgm3, s_isa.density_kgm3);
}

// ---------------------------------------------------------------------------
// T7: ISA+20 density altitude at SL > 0 m
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, NonStandardDay_DensityAltitudeRaisedAtSeaLevel) {
    Atmosphere atm({.delta_temperature_k = 20.f});
    EXPECT_GT(atm.density_altitude_m(0.f), 0.f);
}

// ---------------------------------------------------------------------------
// T8: 50% RH at SL — density lower and speed of sound higher than dry
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, Humidity50pct_DensityLowerSosHigher) {
    Atmosphere dry;
    Atmosphere moist({.relative_humidity_nd = 0.5f});

    AtmosphericState s_dry   = dry.state(0.f);
    AtmosphericState s_moist = moist.state(0.f);

    EXPECT_LT(s_moist.density_kgm3,       s_dry.density_kgm3);
    EXPECT_GT(s_moist.speed_of_sound_mps, s_dry.speed_of_sound_mps);
}

// ---------------------------------------------------------------------------
// T9: 100% RH density < 50% RH density at SL
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, Humidity100pct_DensityLessThan50pct) {
    Atmosphere h50({.relative_humidity_nd = 0.5f});
    Atmosphere h100({.relative_humidity_nd = 1.0f});
    EXPECT_LT(h100.state(0.f).density_kgm3, h50.state(0.f).density_kgm3);
}

// ---------------------------------------------------------------------------
// T10: density_altitude_m increases with delta_temperature_k at fixed altitude
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, DensityAltitude_IncreasesWithWarmerDay) {
    Atmosphere cold({.delta_temperature_k = -10.f});
    Atmosphere std_isa;
    Atmosphere warm({.delta_temperature_k = 20.f});

    float h_cold = cold.density_altitude_m(3000.f);
    float h_std  = std_isa.density_altitude_m(3000.f);
    float h_warm = warm.density_altitude_m(3000.f);

    EXPECT_LT(h_cold, h_std);
    EXPECT_LT(h_std,  h_warm);
}

// ---------------------------------------------------------------------------
// T11: JSON round-trip recovers identical AtmosphericState at 3 000 m
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, JsonRoundTrip_RecoversSameStateAt3000m) {
    Atmosphere original({.delta_temperature_k = 15.f, .relative_humidity_nd = 0.3f});

    nlohmann::json j = original.serializeJson();
    Atmosphere restored;
    restored.deserializeJson(j);

    AtmosphericState s1 = original.state(3000.f);
    AtmosphericState s2 = restored.state(3000.f);

    EXPECT_FLOAT_EQ(s1.temperature_k,      s2.temperature_k);
    EXPECT_FLOAT_EQ(s1.pressure_pa,        s2.pressure_pa);
    EXPECT_FLOAT_EQ(s1.density_kgm3,       s2.density_kgm3);
    EXPECT_FLOAT_EQ(s1.speed_of_sound_mps, s2.speed_of_sound_mps);
    EXPECT_FLOAT_EQ(s1.density_altitude_m, s2.density_altitude_m);
}

// ---------------------------------------------------------------------------
// T12: schema version mismatch throws std::runtime_error
// ---------------------------------------------------------------------------
TEST(AtmosphereTest, SchemaVersionMismatch_Throws) {
    Atmosphere atm;
    nlohmann::json j = atm.serializeJson();
    j["schema_version"] = 99;
    EXPECT_THROW(atm.deserializeJson(j), std::runtime_error);
}

// ---------------------------------------------------------------------------
// LS-T6 / OQ-LS-14: WGS84-input overloads
// ---------------------------------------------------------------------------

namespace {

std::filesystem::path writeConstantUndulationGrid(
    const std::string& fixture_name, float undulation_m)
{
    const auto out =
        std::filesystem::temp_directory_path() / (fixture_name + ".egm2008");
    std::ofstream f(out, std::ios::binary | std::ios::trunc);
    const char magic[8] = {'E', 'G', 'M', '2', '0', '0', '8', '\0'};
    f.write(magic, sizeof(magic));
    const std::uint32_t version = 1;
    f.write(reinterpret_cast<const char*>(&version), sizeof(version));
    const double lat_min = -1.0, lat_max = 1.0, lon_min = -1.0, lon_max = 1.0;
    f.write(reinterpret_cast<const char*>(&lat_min), sizeof(double));
    f.write(reinterpret_cast<const char*>(&lat_max), sizeof(double));
    f.write(reinterpret_cast<const char*>(&lon_min), sizeof(double));
    f.write(reinterpret_cast<const char*>(&lon_max), sizeof(double));
    const std::uint32_t n_lat = 2, n_lon = 2;
    f.write(reinterpret_cast<const char*>(&n_lat), sizeof(std::uint32_t));
    f.write(reinterpret_cast<const char*>(&n_lon), sizeof(std::uint32_t));
    const std::vector<float> values(4, undulation_m);
    f.write(reinterpret_cast<const char*>(values.data()),
            static_cast<std::streamsize>(values.size() * sizeof(float)));
    return out;
}

}  // namespace

TEST(AtmosphereTest, StateAtWgs84_NoGeoid_MatchesStateAtSameNumericInput) {
    Atmosphere atm;
    const float h_input = 1500.0f;
    const auto a = atm.state(h_input);
    const auto b = atm.state_at_wgs84_m(0.0, 0.0, h_input, /*geoid=*/nullptr);
    EXPECT_FLOAT_EQ(a.density_kgm3, b.density_kgm3);
    EXPECT_FLOAT_EQ(a.pressure_pa,  b.pressure_pa);
    EXPECT_FLOAT_EQ(a.temperature_k, b.temperature_k);
}

TEST(AtmosphereTest, StateAtWgs84_WithGeoid_AppliesUndulation) {
    // KSBA-like geoid: N = -33 m.  h_WGS84 = -29 m -> h_MSL = -29 - (-33) = +4 m.
    const auto path = writeConstantUndulationGrid("atm_geoid", -33.0f);
    liteaero::geodesy::Egm2008Geoid geoid(path);

    Atmosphere atm;
    const float h_wgs84 = -29.0f;

    const auto with_geoid    = atm.state_at_wgs84_m(0.0, 0.0, h_wgs84, &geoid);
    const auto reference_msl = atm.state(/*h_msl=*/4.0f);

    EXPECT_NEAR(with_geoid.density_kgm3, reference_msl.density_kgm3, 1e-5f);
    EXPECT_NEAR(with_geoid.pressure_pa,  reference_msl.pressure_pa,  1e-1f);
    EXPECT_NEAR(with_geoid.temperature_k, reference_msl.temperature_k, 1e-3f);
}

TEST(AtmosphereTest, DensityAtWgs84_WithGeoid_MatchesStateAtWgs84) {
    const auto path = writeConstantUndulationGrid("atm_density", -33.0f);
    liteaero::geodesy::Egm2008Geoid geoid(path);

    Atmosphere atm;
    const float h_wgs84 = -29.0f;
    EXPECT_FLOAT_EQ(
        atm.density_at_wgs84_m(0.0, 0.0, h_wgs84, &geoid),
        atm.state_at_wgs84_m(0.0, 0.0, h_wgs84, &geoid).density_kgm3);
}
