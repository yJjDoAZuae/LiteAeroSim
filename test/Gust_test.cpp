#include "environment/Gust.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static GustConfig makeVerticalGust(float amplitude_mps, float gradient_dist_m) {
    GustConfig cfg;
    cfg.amplitude_mps   = amplitude_mps;
    cfg.gradient_dist_m = gradient_dist_m;
    cfg.direction_body  = Eigen::Vector3f{0.f, 0.f, 1.f};  // vertical (body z)
    return cfg;
}

// ---------------------------------------------------------------------------
// T1: before trigger time, step() returns {0,0,0}
// ---------------------------------------------------------------------------
TEST(GustTest, BeforeTrigger_ReturnsZero) {
    Gust gust;
    gust.trigger(makeVerticalGust(10.f, 50.f), 5.0);

    for (double t = 0.0; t < 5.0; t += 0.1) {
        Eigen::Vector3f v = gust.step(t, 50.f);
        EXPECT_FLOAT_EQ(v.norm(), 0.f) << "t=" << t;
    }
    EXPECT_FALSE(gust.is_active());
}

// ---------------------------------------------------------------------------
// T2: at t0 + H/Va (half-gust), output magnitude == Vg within 0.1%
// 1-cosine: v(t) = Vg/2 * (1 - cos(pi*Va*(t-t0)/H))
// Peak at t - t0 = H/Va: v = Vg/2 * (1 - cos(pi)) = Vg/2 * 2 = Vg
// ---------------------------------------------------------------------------
TEST(GustTest, AtHalfGust_PeakEqualsAmplitude) {
    const float Vg = 10.f;
    const float H  = 50.f;
    const float Va = 50.f;
    const double t0 = 2.0;

    Gust gust;
    gust.trigger(makeVerticalGust(Vg, H), t0);

    double t_peak = t0 + static_cast<double>(H) / Va;
    Eigen::Vector3f v = gust.step(t_peak, Va);
    EXPECT_NEAR(v.norm(), Vg, Vg * 1e-3f);
}

// ---------------------------------------------------------------------------
// T3: after t0 + 2H/Va (end of gust), step() returns {0,0,0}
// ---------------------------------------------------------------------------
TEST(GustTest, AfterGust_ReturnsZero) {
    const float H  = 50.f;
    const float Va = 50.f;
    const double t0 = 1.0;

    Gust gust;
    gust.trigger(makeVerticalGust(10.f, H), t0);

    double t_end = t0 + 2.0 * H / Va + 0.01;  // just past the end
    // Step through to end of gust
    for (double t = t0; t <= t_end; t += 0.001) {
        gust.step(t, Va);
    }

    // One step after
    Eigen::Vector3f v = gust.step(t_end + 0.5, Va);
    EXPECT_FLOAT_EQ(v.norm(), 0.f);
}

// ---------------------------------------------------------------------------
// T4: is_active() state machine
// ---------------------------------------------------------------------------
TEST(GustTest, IsActive_TrueOnlyDuringGust) {
    const float H  = 30.f;
    const float Va = 60.f;
    const double t0 = 1.0;

    Gust gust;
    gust.trigger(makeVerticalGust(5.f, H), t0);

    // Before trigger
    gust.step(0.5, Va);
    EXPECT_FALSE(gust.is_active());

    // At trigger
    gust.step(t0, Va);
    EXPECT_TRUE(gust.is_active());

    // Mid-gust
    gust.step(t0 + H / Va, Va);
    EXPECT_TRUE(gust.is_active());

    // After gust
    double t_after = t0 + 2.0 * H / Va + 1.0;
    gust.step(t_after, Va);
    EXPECT_FALSE(gust.is_active());
}

// ---------------------------------------------------------------------------
// T5: numerical integral of gust profile equals Vg * H / Va within 0.1%
// Analytical: int_0^{2H/Va} Vg/2*(1-cos(pi*Va*t/H)) dt = Vg * H / Va
// ---------------------------------------------------------------------------
TEST(GustTest, NumericalIntegral_MatchesAnalytical) {
    const float  Vg  = 8.f;
    const float  H   = 40.f;
    const float  Va  = 40.f;
    const double t0  = 0.0;
    const double dt  = 1e-4;

    Gust gust;
    gust.trigger(makeVerticalGust(Vg, H), t0);

    double integral = 0.0;
    double t_end = t0 + 2.0 * H / Va;
    for (double t = t0; t <= t_end; t += dt) {
        Eigen::Vector3f v = gust.step(t, Va);
        integral += static_cast<double>(v.norm()) * dt;
    }

    double expected = static_cast<double>(Vg) * H / Va;
    EXPECT_NEAR(integral, expected, expected * 1e-3);
}

// ---------------------------------------------------------------------------
// T6: reset() immediately returns is_active()==false; subsequent step returns zero
// ---------------------------------------------------------------------------
TEST(GustTest, Reset_DisarmsImmediately) {
    Gust gust;
    gust.trigger(makeVerticalGust(10.f, 50.f), 0.0);

    // Activate
    gust.step(0.0, 50.f);
    ASSERT_TRUE(gust.is_active());

    gust.reset();
    EXPECT_FALSE(gust.is_active());
    EXPECT_FLOAT_EQ(gust.step(0.5, 50.f).norm(), 0.f);
}
