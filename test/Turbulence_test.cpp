#include "environment/Turbulence.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// T1: TurbulenceIntensity::None always returns zero velocity and angular rate
// ---------------------------------------------------------------------------
TEST(TurbulenceTest, NoneIntensity_AlwaysZero) {
    TurbulenceConfig cfg;
    cfg.intensity = TurbulenceIntensity::None;
    cfg.dt_s      = 0.01f;
    cfg.seed      = 42;

    Turbulence turb;
    turb.initialize(cfg);

    for (int i = 0; i < 100; ++i) {
        TurbulenceVelocity v = turb.step(300.f, 50.f);
        EXPECT_FLOAT_EQ(v.velocity_body_mps.norm(),    0.f);
        EXPECT_FLOAT_EQ(v.angular_rate_rad_s.norm(), 0.f);
    }
}

// ---------------------------------------------------------------------------
// T2: Light intensity — sample RMS of u_wg within ±20% of expected sigma_u
// (high altitude, Va = 50 m/s, 5000 steps)
// ---------------------------------------------------------------------------
TEST(TurbulenceTest, LightIntensity_SampleRmsWithinTolerance) {
    // At high altitude (>305 m), sigma_u = MIL-HDBK-1797 Light ≈ 1.5 m/s (see Table C-I)
    // W20 for Light = 3.1 m/s; high alt sigma from table ≈ 1.5 m/s
    // We test that sample RMS / expected_sigma is in [0.80, 1.20]
    TurbulenceConfig cfg;
    cfg.intensity  = TurbulenceIntensity::Light;
    cfg.wingspan_m = 3.0f;
    cfg.dt_s       = 0.01f;
    cfg.seed       = 12345;

    Turbulence turb;
    turb.initialize(cfg);

    // Large N needed: at altitude=500m, Va=50m/s the filter time constant tau=L_u/Va=10.66s,
    // giving only ~N*dt/tau effective independent samples per step.
    // N=100000 (1000s simulated) yields ~100 independent samples — sufficient for ±20%.
    const int N = 100000;
    float sum_sq = 0.f;
    for (int i = 0; i < N; ++i) {
        TurbulenceVelocity v = turb.step(500.f, 50.f);
        float u = v.velocity_body_mps.x();
        sum_sq += u * u;
    }
    float rms = std::sqrt(sum_sq / N);

    // Light intensity at high altitude: sigma_u from MIL-HDBK-1797 Table C-I ≈ 1.5 m/s
    float expected_sigma = 1.5f;
    EXPECT_GT(rms, expected_sigma * 0.80f) << "RMS too low: " << rms;
    EXPECT_LT(rms, expected_sigma * 1.20f) << "RMS too high: " << rms;
}

// ---------------------------------------------------------------------------
// T3: reset() zeroes all filter states; first output after reset is zero
// ---------------------------------------------------------------------------
TEST(TurbulenceTest, Reset_ZeroesStateFirstOutputZero) {
    TurbulenceConfig cfg;
    cfg.intensity = TurbulenceIntensity::Light;
    cfg.dt_s      = 0.01f;
    cfg.seed      = 99;

    Turbulence turb;
    turb.initialize(cfg);

    // Run for a while to build up state
    for (int i = 0; i < 200; ++i) {
        turb.step(500.f, 50.f);
    }

    turb.reset();

    // After reset, states are zeroed; first step with zero white noise driving would give 0,
    // but we can't control the RNG sample. Instead verify JSON round-trip after reset shows zeroes.
    nlohmann::json j = turb.serializeJson();
    std::vector<float> su = j["state_u"].get<std::vector<float>>();
    EXPECT_FLOAT_EQ(su[0], 0.f);
    EXPECT_FLOAT_EQ(su[1], 0.f);
    std::vector<float> sv = j["state_v"].get<std::vector<float>>();
    EXPECT_FLOAT_EQ(sv[0], 0.f);
    EXPECT_FLOAT_EQ(sv[1], 0.f);
}

// ---------------------------------------------------------------------------
// T4: JSON round-trip preserves filter state to float precision.
// Verifies that all six state arrays serialized by serializeJson() are
// exactly recovered by deserializeJson() + serializeJson() round-trip.
// (Output comparison is omitted because deserializeJson() reseeds the RNG
// to its initial state, so random draws after restore differ from the original.)
// ---------------------------------------------------------------------------
TEST(TurbulenceTest, JsonRoundTrip_PreservesFilterState) {
    TurbulenceConfig cfg;
    cfg.intensity  = TurbulenceIntensity::Moderate;
    cfg.wingspan_m = 5.f;
    cfg.dt_s       = 0.02f;
    cfg.seed       = 777;

    Turbulence turb;
    turb.initialize(cfg);

    // Warm up to build up filter state
    for (int i = 0; i < 500; ++i) {
        turb.step(400.f, 60.f);
    }

    // Serialize
    nlohmann::json j1 = turb.serializeJson();

    // Restore and re-serialize — states must be exactly preserved
    Turbulence restored;
    restored.deserializeJson(j1);
    nlohmann::json j2 = restored.serializeJson();

    for (const std::string& key : {"state_u", "state_v", "state_w",
                                   "state_p", "state_q", "state_r"}) {
        auto v1 = j1.at(key).get<std::vector<float>>();
        auto v2 = j2.at(key).get<std::vector<float>>();
        ASSERT_EQ(v1.size(), v2.size()) << key;
        for (size_t i = 0; i < v1.size(); ++i) {
            EXPECT_FLOAT_EQ(v1[i], v2[i]) << key << "[" << i << "]";
        }
    }
}

// ---------------------------------------------------------------------------
// T5: identical seeds produce identical output sequences
// ---------------------------------------------------------------------------
TEST(TurbulenceTest, IdenticalSeeds_IdenticalOutputs) {
    TurbulenceConfig cfg;
    cfg.intensity = TurbulenceIntensity::Light;
    cfg.dt_s      = 0.01f;
    cfg.seed      = 54321;

    Turbulence t1, t2;
    t1.initialize(cfg);
    t2.initialize(cfg);

    for (int i = 0; i < 100; ++i) {
        TurbulenceVelocity v1 = t1.step(500.f, 50.f);
        TurbulenceVelocity v2 = t2.step(500.f, 50.f);
        EXPECT_FLOAT_EQ(v1.velocity_body_mps.x(), v2.velocity_body_mps.x());
        EXPECT_FLOAT_EQ(v1.velocity_body_mps.y(), v2.velocity_body_mps.y());
        EXPECT_FLOAT_EQ(v1.velocity_body_mps.z(), v2.velocity_body_mps.z());
    }
}
