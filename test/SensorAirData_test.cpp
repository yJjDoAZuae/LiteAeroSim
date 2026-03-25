#define _USE_MATH_DEFINES
#include "sensor/SensorAirData.hpp"
#include "environment/AtmosphericState.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <gtest/gtest.h>
#include <numeric>
#include <vector>

using liteaero::simulation::AirDataMeasurement;
using liteaero::simulation::AirDataConfig;
using liteaero::simulation::SensorAirData;
using liteaero::simulation::AtmosphericState;

namespace {

static constexpr float kT0    = 288.15f;
static constexpr float kP0    = 101325.0f;
static constexpr float kRho0  = 1.225f;
static constexpr float kA0    = 340.294f;
static constexpr float kRd    = 287.058f;
static constexpr float kGamma = 1.4f;
static constexpr float kG0    = 9.80665f;
static constexpr float kL     = -0.0065f;  // troposphere lapse rate (K/m)

// ISA state at sea level.
AtmosphericState isa_sl() {
    return { kT0, kP0, kRho0, kA0, 0.0f, 0.0f };
}

// ISA state at geometric altitude h_m (troposphere).
AtmosphericState isa(float h_m) {
    const float T   = kT0 + kL * h_m;
    const float P   = kP0 * std::pow(T / kT0, -kG0 / (kRd * kL));
    const float rho = P / (kRd * T);
    const float a   = std::sqrt(kGamma * kRd * T);
    return { T, P, rho, a, 0.0f, 0.0f };
}

// Warm-day atmosphere: constant +delta_t_k offset throughout the column.
// Surface pressure is still P0; pressure at altitude is physically higher than ISA.
AtmosphericState isa_warm(float h_m, float delta_t_k) {
    const float T_sfc_warm = kT0 + delta_t_k;
    const float Tw         = T_sfc_warm + kL * h_m;
    const float P          = kP0 * std::pow(Tw / T_sfc_warm, -kG0 / (kRd * kL));
    const float rho        = P / (kRd * Tw);
    const float a          = std::sqrt(kGamma * kRd * Tw);
    return { Tw, P, rho, a, 0.0f, 0.0f };
}

// Non-standard surface pressure: P_s at h_m = P_ISA(h_m) * p_sfc_pa / P0.
AtmosphericState isa_nonstd_pressure(float h_m, float p_sfc_pa) {
    AtmosphericState s    = isa(h_m);
    const float scale     = p_sfc_pa / kP0;
    s.pressure_pa        *= scale;
    s.density_kgm3       *= scale;
    return s;
}

nlohmann::json zero_config(float dt_s = 0.01f) {
    return nlohmann::json{{"dt_s", dt_s}};
}

Eigen::Vector3f level_airspeed(float tas_mps) {
    return Eigen::Vector3f(tas_mps, 0.0f, 0.0f);
}

float sample_stddev(const std::vector<float>& v) {
    const float mean = std::accumulate(v.begin(), v.end(), 0.0f) / static_cast<float>(v.size());
    float var = 0.0f;
    for (float x : v) var += (x - mean) * (x - mean);
    return std::sqrt(var / static_cast<float>(v.size() - 1));
}

float sample_variance(const std::vector<float>& v) {
    const float sd = sample_stddev(v);
    return sd * sd;
}

}  // namespace

class SensorAirDataTest : public ::testing::Test {};

// T1: Sea-level ISA, zero noise, zero lag, TAS=20: all airspeeds ≈ 20 m/s
TEST_F(SensorAirDataTest, SeaLevelISA_ZeroNoise_ZeroLag_TAS20_AllAirspeedsEqual) {
    SensorAirData sensor(zero_config());
    const auto meas = sensor.step(level_airspeed(20.0f), isa_sl());
    EXPECT_NEAR(meas.ias_mps, 20.0f, 0.1f);
    EXPECT_NEAR(meas.cas_mps, 20.0f, 0.1f);
    EXPECT_NEAR(meas.eas_mps, 20.0f, 0.1f);
    EXPECT_NEAR(meas.tas_mps, 20.0f, 0.1f);
}

// T2: 3000 m ISA, zero noise, zero lag, TAS=40: IAS < TAS, CAS < TAS, EAS < TAS
TEST_F(SensorAirDataTest, Altitude3000m_ZeroNoise_ZeroLag_AllAirspeedsLessThanTAS) {
    SensorAirData sensor(zero_config());
    const auto meas = sensor.step(level_airspeed(40.0f), isa(3000.0f));
    EXPECT_LT(meas.ias_mps, meas.tas_mps);
    EXPECT_LT(meas.cas_mps, meas.tas_mps);
    EXPECT_LT(meas.eas_mps, meas.tas_mps);
}

// T3: Barometric altitude matches geometric within 10 m at 3000 m ISA
TEST_F(SensorAirDataTest, BaroAltitude_ISA_3000m_WithinTenMeters) {
    SensorAirData sensor(zero_config());
    const auto meas = sensor.step(level_airspeed(40.0f), isa(3000.0f));
    EXPECT_NEAR(meas.baro_altitude_m, 3000.0f, 10.0f);
}

// T4: Warm day — altimeter reads low relative to geometric (geometric altitude > baro altitude)
TEST_F(SensorAirDataTest, BaroAltitude_WarmDay_ReadsHighRelativeToGeometric) {
    SensorAirData sensor(zero_config());
    const float geo_alt_m = 3000.0f;
    // On a warm day the atmosphere is "puffed up": pressure at a given geometric altitude
    // is higher than ISA → altimeter inverts this to a lower indicated altitude.
    const auto meas = sensor.step(level_airspeed(40.0f), isa_warm(geo_alt_m, 20.0f));
    EXPECT_GT(geo_alt_m, meas.baro_altitude_m);
}

// T5: IAS sample std-dev matches σ_qc / (ρ₀ · V_IAS) within 20 %
TEST_F(SensorAirDataTest, DifferentialPressureNoise_IASStddev_MatchesPropagation) {
    const float sigma_qc = 50.0f;
    const float tas      = 30.0f;
    nlohmann::json cfg   = zero_config();
    cfg["differential_pressure_noise_pa"] = sigma_qc;
    cfg["seed"]                           = uint32_t{42};
    SensorAirData sensor(cfg);

    std::vector<float> samples;
    samples.reserve(1000);
    for (int i = 0; i < 1000; ++i)
        samples.push_back(sensor.step(level_airspeed(tas), isa_sl()).ias_mps);

    // σ_IAS ≈ σ_qc / (ρ₀ · V_IAS); at sea-level ISA, V_IAS ≈ TAS
    const float expected_stddev = sigma_qc / (kRho0 * tas);
    EXPECT_NEAR(sample_stddev(samples), expected_stddev, expected_stddev * 0.20f);
}

// T6: Barometric altitude sample std-dev matches σ_Ps / (ρ₀ · g₀) within 20 %
TEST_F(SensorAirDataTest, StaticPressureNoise_BaroAltStddev_MatchesPropagation) {
    const float sigma_ps = 50.0f;
    nlohmann::json cfg   = zero_config();
    cfg["static_pressure_noise_pa"] = sigma_ps;
    cfg["seed"]                     = uint32_t{42};
    SensorAirData sensor(cfg);

    std::vector<float> samples;
    samples.reserve(1000);
    for (int i = 0; i < 1000; ++i)
        samples.push_back(sensor.step(level_airspeed(30.0f), isa_sl()).baro_altitude_m);

    // At sea level: σ_h ≈ Rd·T0/(g0·P0) · σ_Ps ≈ σ_Ps / (ρ₀ · g₀)
    const float expected_stddev = sigma_ps / (kRho0 * kG0);
    EXPECT_NEAR(sample_stddev(samples), expected_stddev, expected_stddev * 0.20f);
}

// T7: Nonzero lag reduces IAS sample variance relative to no-lag case
TEST_F(SensorAirDataTest, Lag_ReducesVariance) {
    const float sigma_qc = 100.0f;
    const float tau      = 0.5f;
    const float tas      = 30.0f;
    const auto  atm      = isa_sl();

    nlohmann::json cfg_no_lag = zero_config();
    cfg_no_lag["differential_pressure_noise_pa"] = sigma_qc;
    cfg_no_lag["seed"]                           = uint32_t{42};
    SensorAirData sensor_no_lag(cfg_no_lag);

    nlohmann::json cfg_lagged = cfg_no_lag;
    cfg_lagged["differential_pressure_lag_tau_s"] = tau;
    SensorAirData sensor_lagged(cfg_lagged);

    // Warmup to allow lagged sensor to reach steady-state
    for (int i = 0; i < 500; ++i) {
        sensor_no_lag.step(level_airspeed(tas), atm);
        sensor_lagged.step(level_airspeed(tas), atm);
    }

    std::vector<float> v_no_lag, v_lagged;
    v_no_lag.reserve(1000);
    v_lagged.reserve(1000);
    for (int i = 0; i < 1000; ++i) {
        v_no_lag.push_back(sensor_no_lag.step(level_airspeed(tas), atm).ias_mps);
        v_lagged.push_back(sensor_lagged.step(level_airspeed(tas), atm).ias_mps);
    }

    EXPECT_LT(sample_variance(v_lagged), sample_variance(v_no_lag));
}

// T8: Zero noise, zero lag: output is deterministic and matches noiseless formulas
TEST_F(SensorAirDataTest, ZeroNoise_ZeroLag_Deterministic_MatchesNoiselessDerived) {
    SensorAirData sensor(zero_config());
    const float tas = 30.0f;
    const auto  atm = isa_sl();
    const auto  meas = sensor.step(level_airspeed(tas), atm);

    const float M      = tas / kA0;
    const float qc     = kP0 * (std::pow(1.0f + 0.2f * M * M, 3.5f) - 1.0f);
    const float exp_ias  = std::sqrt(2.0f * qc / kRho0);
    const float exp_mach = std::sqrt(5.0f * (std::pow(qc / kP0 + 1.0f, 2.0f / 7.0f) - 1.0f));
    const float exp_tas  = exp_mach * kA0;

    EXPECT_NEAR(meas.ias_mps, exp_ias,  1e-3f);
    EXPECT_NEAR(meas.mach_nd, exp_mach, 1e-5f);
    EXPECT_NEAR(meas.tas_mps, exp_tas,  1e-3f);
    EXPECT_NEAR(meas.oat_k,   kT0,      1e-3f);

    // A second fresh sensor with same config produces bit-identical output
    SensorAirData sensor2(zero_config());
    const auto meas2 = sensor2.step(level_airspeed(tas), atm);
    EXPECT_FLOAT_EQ(meas.ias_mps,         meas2.ias_mps);
    EXPECT_FLOAT_EQ(meas.mach_nd,         meas2.mach_nd);
    EXPECT_FLOAT_EQ(meas.baro_altitude_m, meas2.baro_altitude_m);
}

// T9: After reset(), first step output equals zero-lag noiseless value
TEST_F(SensorAirDataTest, Reset_ZeroesFilterState) {
    nlohmann::json cfg = zero_config();
    cfg["differential_pressure_lag_tau_s"] = 0.2f;
    cfg["static_pressure_lag_tau_s"]       = 0.2f;
    SensorAirData sensor(cfg);

    const auto v   = level_airspeed(30.0f);
    const auto atm = isa(3000.0f);

    // Run to steady state (5 × τ / dt = 100 steps; use 300 for extra margin)
    for (int i = 0; i < 300; ++i) sensor.step(v, atm);

    sensor.reset();
    const auto meas = sensor.step(v, atm);

    // Expected: zero-lag noiseless value from an ideal (τ=0) sensor
    SensorAirData ideal(zero_config());
    const auto expected = ideal.step(v, atm);

    EXPECT_NEAR(meas.ias_mps,         expected.ias_mps,         1e-3f);
    EXPECT_NEAR(meas.baro_altitude_m, expected.baro_altitude_m, 0.1f);
}

// T10: Two instances with the same seed produce bit-identical output
TEST_F(SensorAirDataTest, IdenticalSeeds_IdenticalOutputs) {
    nlohmann::json cfg = zero_config();
    cfg["differential_pressure_noise_pa"] = 50.0f;
    cfg["static_pressure_noise_pa"]       = 50.0f;
    cfg["oat_noise_k"]                    = 1.0f;
    cfg["seed"]                           = uint32_t{12345};

    SensorAirData s1(cfg);
    SensorAirData s2(cfg);

    const auto v   = level_airspeed(30.0f);
    const auto atm = isa(2000.0f);

    for (int i = 0; i < 100; ++i) {
        const auto m1 = s1.step(v, atm);
        const auto m2 = s2.step(v, atm);
        EXPECT_FLOAT_EQ(m1.ias_mps,         m2.ias_mps);
        EXPECT_FLOAT_EQ(m1.cas_mps,         m2.cas_mps);
        EXPECT_FLOAT_EQ(m1.eas_mps,         m2.eas_mps);
        EXPECT_FLOAT_EQ(m1.tas_mps,         m2.tas_mps);
        EXPECT_FLOAT_EQ(m1.mach_nd,         m2.mach_nd);
        EXPECT_FLOAT_EQ(m1.baro_altitude_m, m2.baro_altitude_m);
        EXPECT_FLOAT_EQ(m1.oat_k,           m2.oat_k);
    }
}

// T11: JSON round-trip preserves filter state; next step is identical
TEST_F(SensorAirDataTest, JsonRoundTrip_PreservesFilterState) {
    nlohmann::json cfg = zero_config();
    cfg["differential_pressure_noise_pa"]  = 30.0f;
    cfg["static_pressure_noise_pa"]        = 30.0f;
    cfg["differential_pressure_lag_tau_s"] = 0.1f;
    cfg["static_pressure_lag_tau_s"]       = 0.1f;
    cfg["seed"]                            = uint32_t{777};

    SensorAirData s1(cfg);
    const auto v   = level_airspeed(30.0f);
    const auto atm = isa(1500.0f);

    for (int i = 0; i < 50; ++i) s1.step(v, atm);

    const nlohmann::json snap = s1.serializeJson();

    SensorAirData s2(cfg);
    s2.deserializeJson(snap);

    const auto m1 = s1.step(v, atm);
    const auto m2 = s2.step(v, atm);
    EXPECT_FLOAT_EQ(m1.ias_mps,         m2.ias_mps);
    EXPECT_FLOAT_EQ(m1.cas_mps,         m2.cas_mps);
    EXPECT_FLOAT_EQ(m1.eas_mps,         m2.eas_mps);
    EXPECT_FLOAT_EQ(m1.tas_mps,         m2.tas_mps);
    EXPECT_FLOAT_EQ(m1.mach_nd,         m2.mach_nd);
    EXPECT_FLOAT_EQ(m1.baro_altitude_m, m2.baro_altitude_m);
    EXPECT_FLOAT_EQ(m1.oat_k,           m2.oat_k);
}

// T12: Proto round-trip preserves filter state; next step is identical
TEST_F(SensorAirDataTest, ProtoRoundTrip_PreservesFilterState) {
    nlohmann::json cfg = zero_config();
    cfg["differential_pressure_noise_pa"]  = 30.0f;
    cfg["static_pressure_noise_pa"]        = 30.0f;
    cfg["differential_pressure_lag_tau_s"] = 0.1f;
    cfg["static_pressure_lag_tau_s"]       = 0.1f;
    cfg["seed"]                            = uint32_t{888};

    SensorAirData s1(cfg);
    const auto v   = level_airspeed(30.0f);
    const auto atm = isa(1500.0f);

    for (int i = 0; i < 50; ++i) s1.step(v, atm);

    const std::vector<uint8_t> bytes = s1.serializeProto();

    SensorAirData s2(cfg);
    s2.deserializeProto(bytes);

    const auto m1 = s1.step(v, atm);
    const auto m2 = s2.step(v, atm);
    EXPECT_FLOAT_EQ(m1.ias_mps,         m2.ias_mps);
    EXPECT_FLOAT_EQ(m1.baro_altitude_m, m2.baro_altitude_m);
    EXPECT_FLOAT_EQ(m1.oat_k,           m2.oat_k);
}

// T13: deserializeJson with wrong schema_version throws std::runtime_error
TEST_F(SensorAirDataTest, SchemaVersionMismatch_Throws) {
    SensorAirData sensor(zero_config());
    nlohmann::json snap = sensor.serializeJson();
    snap["schema_version"] = 99;
    EXPECT_THROW(sensor.deserializeJson(snap), std::runtime_error);
}

// T14: Fuselage crossflow pressure error with known alpha matches formula ΔP_geo/(ρ₀·g₀)
//
// Test geometry: sea-level ISA (ideal baro altitude = 0 m), zero noise, zero lag.
// The sensor with static_port_angle_rad = 30° adds ΔP_geo to the static channel.
// The baro offset from the ideal (zero-error) 0 m reading matches -ΔP_geo/(ρ₀·g₀) within 0.1%.
TEST_F(SensorAirDataTest, FuselagePressureError_KnownAlpha_MatchesFormula) {
    const float phi = static_cast<float>(M_PI) / 6.0f;  // 30°
    nlohmann::json cfg = zero_config();
    cfg["static_port_angle_rad"] = phi;
    SensorAirData sensor(cfg);

    const float tas       = 30.0f;
    const float alpha_rad = 5.0f * static_cast<float>(M_PI) / 180.0f;
    // Body-frame airspeed with angle of attack: u=cos α·V, w=sin α·V, v=0
    const Eigen::Vector3f v_body(tas * std::cos(alpha_rad), 0.0f, tas * std::sin(alpha_rad));
    const auto atm = isa_sl();

    const auto meas_error = sensor.step(v_body, atm);

    // Compute ΔP_geo from the two-port crosslinked formula
    const float sin_alpha   = std::sin(alpha_rad);
    const float sin_beta    = 0.0f;
    const float delta       = std::atan2(sin_beta, -sin_alpha);
    const float q_cross     = 0.5f * kRho0 * tas * tas
                              * (sin_alpha * sin_alpha + sin_beta * sin_beta);
    const float cp_bar      = -1.0f - 2.0f * std::cos(2.0f * phi) * std::cos(2.0f * delta);
    const float delta_p_geo = cp_bar * q_cross;

    // At sea-level ISA the zero-error baro altitude is 0 m.
    // Lower Ps → higher indicated altitude, so offset ≈ -ΔP_geo / (ρ₀·g₀).
    const float expected_offset = -delta_p_geo / (kRho0 * kG0);
    const float actual_offset   = meas_error.baro_altitude_m;   // ideal = 0 m at sea level

    EXPECT_NEAR(actual_offset, expected_offset, std::abs(expected_offset) * 0.001f + 0.01f);
}

// T15: Crosslinked ports at φ=30° cancel β error for β up to 15°
TEST_F(SensorAirDataTest, CrosslinkedPorts_BetaCancelled_AtThirtyDegrees) {
    const float phi = static_cast<float>(M_PI) / 6.0f;  // 30°
    nlohmann::json cfg = zero_config();
    cfg["static_port_angle_rad"] = phi;
    SensorAirData sensor(cfg);

    const float tas = 30.0f;
    for (float beta_deg : {5.0f, 10.0f, 15.0f}) {
        const float beta_rad = beta_deg * static_cast<float>(M_PI) / 180.0f;
        // Pure sideslip: u=cos β·V, v=sin β·V, w=0
        const Eigen::Vector3f v_body(tas * std::cos(beta_rad), tas * std::sin(beta_rad), 0.0f);
        const auto meas = sensor.step(v_body, isa_sl());
        // β error is fully cancelled at φ=30°; reading should be sea level ≈ 0 m
        EXPECT_NEAR(meas.baro_altitude_m, 0.0f, 1.0f)
            << "beta_deg=" << beta_deg;
    }
}

// T16: Correctly set Kollsman cancels non-standard surface pressure error
TEST_F(SensorAirDataTest, Kollsman_CorrectQNH_CancelsNonStandardPressure) {
    const float delta_p_sfc = 500.0f;
    const float p_sfc       = kP0 + delta_p_sfc;
    nlohmann::json cfg = zero_config();
    cfg["initial_kollsman_pa"] = p_sfc;

    SensorAirData sensor(cfg);
    const float geo_alt = 3000.0f;
    const auto  meas    = sensor.step(level_airspeed(40.0f), isa_nonstd_pressure(geo_alt, p_sfc));

    EXPECT_NEAR(meas.baro_altitude_m, geo_alt, 10.0f);
}

// T17: Stale Kollsman produces expected underread of ≈41 m for +500 Pa surface pressure deviation
TEST_F(SensorAirDataTest, Kollsman_StaleQNH_ProducesExpectedError) {
    const float delta_p_sfc = 500.0f;
    const float p_sfc       = kP0 + delta_p_sfc;
    // Stale Kollsman: stays at ISA standard P0; use sea level so linearised formula is exact
    SensorAirData sensor(zero_config());
    const float geo_alt = 0.0f;
    const auto  meas    = sensor.step(level_airspeed(40.0f), isa_nonstd_pressure(geo_alt, p_sfc));

    // Δh ≈ Rd·T0/(g0·P0) · ΔP_sfc  (negative = underread on high-pressure day)
    const float expected_error = -(kRd * kT0 / (kG0 * kP0)) * delta_p_sfc;
    EXPECT_NEAR(meas.baro_altitude_m - geo_alt, expected_error, 2.0f);
}

// T18: setKollsman() takes effect on the very next step()
TEST_F(SensorAirDataTest, SetKollsman_UpdatesNextStep) {
    const float delta_p_sfc = 500.0f;
    const float p_sfc       = kP0 + delta_p_sfc;
    const float geo_alt     = 0.0f;
    const auto  atm         = isa_nonstd_pressure(geo_alt, p_sfc);
    const auto  v           = level_airspeed(40.0f);

    SensorAirData sensor(zero_config());

    // 10 steps with stale Kollsman (P0)
    AirDataMeasurement last_stale{};
    for (int i = 0; i < 10; ++i) last_stale = sensor.step(v, atm);

    sensor.setKollsman(p_sfc);
    const auto meas_corrected = sensor.step(v, atm);

    // Shift should be ≈ +41 m (stale → correct QNH)
    const float expected_shift = (kRd * kT0 / (kG0 * kP0)) * delta_p_sfc;
    EXPECT_NEAR(meas_corrected.baro_altitude_m - last_stale.baro_altitude_m,
                expected_shift, 2.0f);
}

// T19: JSON round-trip preserves Kollsman setting and baro altitude output
TEST_F(SensorAirDataTest, JsonRoundTrip_PreservesKollsman) {
    const float p_sfc   = kP0 + 500.0f;
    const float geo_alt = 3000.0f;
    const auto  atm     = isa_nonstd_pressure(geo_alt, p_sfc);
    const auto  v       = level_airspeed(40.0f);

    SensorAirData s1(zero_config());
    s1.setKollsman(p_sfc);
    for (int i = 0; i < 10; ++i) s1.step(v, atm);

    const nlohmann::json snap = s1.serializeJson();
    SensorAirData s2(zero_config());
    s2.deserializeJson(snap);

    EXPECT_FLOAT_EQ(s2.kollsman_pa(), p_sfc);

    const auto m1 = s1.step(v, atm);
    const auto m2 = s2.step(v, atm);
    EXPECT_FLOAT_EQ(m1.baro_altitude_m, m2.baro_altitude_m);
}
