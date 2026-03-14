#include "environment/Wind.hpp"
#include <gtest/gtest.h>
#include <Eigen/Dense>

using namespace liteaerosim::environment;

// ---------------------------------------------------------------------------
// T1: default-constructed Wind returns {0,0,0} at any altitude
// ---------------------------------------------------------------------------
TEST(WindTest, DefaultConstructed_ReturnsZeroWind) {
    Wind wind;
    Eigen::Vector3f w = wind.wind_NED_mps(0.f);
    EXPECT_FLOAT_EQ(w.x(), 0.f);
    EXPECT_FLOAT_EQ(w.y(), 0.f);
    EXPECT_FLOAT_EQ(w.z(), 0.f);

    w = wind.wind_NED_mps(5000.f);
    EXPECT_FLOAT_EQ(w.norm(), 0.f);
}

// ---------------------------------------------------------------------------
// T2: Constant profile — same wind at all altitudes
// ---------------------------------------------------------------------------
TEST(WindTest, ConstantProfile_SameAtAllAltitudes) {
    WindConfig cfg;
    cfg.direction_NED_unit = Eigen::Vector3f{1.f, 0.f, 0.f};
    cfg.speed_mps          = 5.f;
    cfg.profile            = WindConfig::Profile::Constant;
    Wind wind(cfg);

    auto check = [&](float alt) {
        Eigen::Vector3f w = wind.wind_NED_mps(alt);
        EXPECT_NEAR(w.x(), 5.f, 1e-5f) << "alt=" << alt;
        EXPECT_NEAR(w.y(), 0.f, 1e-5f) << "alt=" << alt;
        EXPECT_NEAR(w.z(), 0.f, 1e-5f) << "alt=" << alt;
    };
    check(0.f);
    check(500.f);
    check(5000.f);
}

// ---------------------------------------------------------------------------
// T3: PowerLaw at reference altitude returns reference speed exactly
// ---------------------------------------------------------------------------
TEST(WindTest, PowerLaw_AtReferenceAltitude_ReturnsReferenceSpeed) {
    WindConfig cfg;
    cfg.direction_NED_unit  = Eigen::Vector3f{0.f, 1.f, 0.f};
    cfg.speed_mps           = 8.f;
    cfg.reference_altitude_m = 10.f;
    cfg.profile             = WindConfig::Profile::PowerLaw;
    cfg.hellmann_alpha_nd   = 0.14f;
    Wind wind(cfg);

    float mag = wind.wind_NED_mps(10.f).norm();
    EXPECT_NEAR(mag, 8.f, 8.f * 1e-5f);
}

// ---------------------------------------------------------------------------
// T4: PowerLaw at 2x reference altitude produces higher magnitude
// ---------------------------------------------------------------------------
TEST(WindTest, PowerLaw_DoubleAltitude_HigherMagnitude) {
    WindConfig cfg;
    cfg.direction_NED_unit  = Eigen::Vector3f{1.f, 0.f, 0.f};
    cfg.speed_mps           = 5.f;
    cfg.reference_altitude_m = 10.f;
    cfg.profile             = WindConfig::Profile::PowerLaw;
    cfg.hellmann_alpha_nd   = 0.14f;
    Wind wind(cfg);

    float mag_ref = wind.wind_NED_mps(10.f).norm();
    float mag_2x  = wind.wind_NED_mps(20.f).norm();
    EXPECT_GT(mag_2x, mag_ref);
}

// ---------------------------------------------------------------------------
// T5: PowerLaw at altitude <= 0 does not crash
// ---------------------------------------------------------------------------
TEST(WindTest, PowerLaw_ZeroOrNegativeAltitude_NoCrash) {
    WindConfig cfg;
    cfg.direction_NED_unit  = Eigen::Vector3f{1.f, 0.f, 0.f};
    cfg.speed_mps           = 5.f;
    cfg.reference_altitude_m = 10.f;
    cfg.profile             = WindConfig::Profile::PowerLaw;
    Wind wind(cfg);

    EXPECT_NO_THROW(wind.wind_NED_mps(0.f));
    EXPECT_NO_THROW(wind.wind_NED_mps(-10.f));
    // Result should be finite
    EXPECT_TRUE(std::isfinite(wind.wind_NED_mps(0.f).norm()));
}

// ---------------------------------------------------------------------------
// T6: JSON round-trip recovers identical wind_NED_mps output
// ---------------------------------------------------------------------------
TEST(WindTest, JsonRoundTrip_RecoversSameOutput) {
    WindConfig cfg;
    cfg.direction_NED_unit  = Eigen::Vector3f{0.707f, 0.707f, 0.f};
    cfg.speed_mps           = 12.f;
    cfg.reference_altitude_m = 50.f;
    cfg.profile             = WindConfig::Profile::PowerLaw;
    cfg.hellmann_alpha_nd   = 0.2f;
    Wind original(cfg);

    nlohmann::json j = original.serializeJson();
    Wind restored;
    restored.deserializeJson(j);

    float alt = 100.f;
    Eigen::Vector3f w1 = original.wind_NED_mps(alt);
    Eigen::Vector3f w2 = restored.wind_NED_mps(alt);
    EXPECT_NEAR(w1.x(), w2.x(), 1e-4f);
    EXPECT_NEAR(w1.y(), w2.y(), 1e-4f);
    EXPECT_NEAR(w1.z(), w2.z(), 1e-4f);
}
