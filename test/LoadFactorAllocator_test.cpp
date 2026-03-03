#define _USE_MATH_DEFINES
#include "aerodynamics/LoadFactorAllocator.hpp"
#include <gtest/gtest.h>
#include <cmath>

// Representative GA aircraft parameters (matching LiftCurveModel_test.cpp).
static LiftCurveParams gaLiftParams() {
    return {5.73f, 1.20f, -1.20f, 0.05f, 0.05f, 0.80f, -0.80f};
}

// Aircraft sizing: chosen so level-flight CL is comfortably within the linear
// region and the stall limit is above 1 g.
//   mass = 1000 kg, S = 16 m², q = 1531 Pa  →  qS ≈ 24 496 N
//   n_1g demand  = m·g = 9 806.65 N  →  CL ≈ 0.40  (linear region, α ≈ 7°)
//   n_max (T=0)  = cl_max·qS/mg ≈ 3.0 g  (stall ceiling with zero thrust)
static const float kMass = 1000.0f;    // kg
static const float kS    =   16.0f;    // m²
static const float kQ    = 1531.0f;    // Pa (≈ 50 m/s at sea level)
static const float kCYb  =   -3.0f;    // C_Yβ (rad⁻¹)
static const float kG    =    9.80665f; // m/s²

struct AllocatorFixture : ::testing::Test {
    LiftCurveModel     lift{gaLiftParams()};
    LoadFactorAllocator alloc{lift, kS, kCYb};
};

// ── Zero demand ───────────────────────────────────────────────────────────────

TEST_F(AllocatorFixture, ZeroDemandZeroAngles) {
    // n=0, n_y=0, T=0 → f(0)=0 and g(0)=0 exactly; solver stays at α=β=0.
    LoadFactorInputs in{0.f, 0.f, kQ, 0.f, kMass};
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.alpha_rad, 0.f, 1e-5f);
    EXPECT_NEAR(out.beta_rad,  0.f, 1e-5f);
    EXPECT_EQ(out.alpha_segment, LiftCurveSegment::Linear);
}

// ── Linear region, analytic check ─────────────────────────────────────────────

TEST_F(AllocatorFixture, SmallNLinearRegionT0) {
    // With T=0, linear-region solution: α = n·m·g / (q·S·C_Lα).
    // Use n=1 (1 g).  α ≈ 9807 / (24496·5.73) ≈ 0.1711 rad < α_star ≈ 0.234 rad.
    const float n = 1.0f;
    const float alpha_expected = n * kMass * kG / (kQ * kS * gaLiftParams().cl_alpha);
    ASSERT_LT(alpha_expected, lift.alphaPeak()); // verify we're in pre-stall

    LoadFactorInputs in{n, 0.f, kQ, 0.f, kMass};
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.alpha_rad, alpha_expected, 1e-4f);
    EXPECT_EQ(out.alpha_segment, LiftCurveSegment::Linear);
}

// ── Stall flag ────────────────────────────────────────────────────────────────

TEST_F(AllocatorFixture, PositiveStallClampsAtPeak) {
    // n_max (T=0) ≈ cl_max·qS / (m·g) ≈ 3.0 g.  Use 20 % above ceiling.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);
    const float n_stall   = n_ceiling * 1.2f;

    LoadFactorInputs in{n_stall, 0.f, kQ, 0.f, kMass};
    auto out = alloc.solve(in);
    // α clamped at the pre-stall peak — sits at the incipient/post boundary.
    EXPECT_NEAR(out.alpha_rad, lift.alphaPeak(), 1e-4f);
    EXPECT_EQ(out.alpha_segment, LiftCurveSegment::IncipientStallPositive);
}

TEST_F(AllocatorFixture, NegativeStallClampsAtTrough) {
    // Symmetric to positive stall: n more negative than the negative CL ceiling.
    const float n_ceiling_neg = gaLiftParams().cl_min * kQ * kS / (kMass * kG);
    const float n_stall_neg   = n_ceiling_neg * 1.2f; // 20 % more negative

    LoadFactorInputs in{n_stall_neg, 0.f, kQ, 0.f, kMass};
    auto out = alloc.solve(in);
    // α clamped at the negative trough.
    EXPECT_NEAR(out.alpha_rad, lift.alphaTrough(), 1e-4f);
    EXPECT_EQ(out.alpha_segment, LiftCurveSegment::IncipientStallNegative);
}

// ── Branch tracking ───────────────────────────────────────────────────────────

TEST_F(AllocatorFixture, MonotonicNStaysPreStall) {
    // Step n up from 0 to just below the stall ceiling in 0.1-g increments.
    // Verify that α increases monotonically and stall is never flagged.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);

    float alpha_prev = 0.f;
    for (float n = 0.1f; n < n_ceiling * 0.95f; n += 0.1f) {
        LoadFactorInputs in{n, 0.f, kQ, 0.f, kMass};
        auto out = alloc.solve(in);
        // α must stay in the pre-stall envelope (linear or incipient, not post-stall or separated).
        const auto seg = out.alpha_segment;
        EXPECT_TRUE(seg == LiftCurveSegment::Linear || seg == LiftCurveSegment::IncipientStallPositive)
            << "unexpected segment at n=" << n;
        EXPECT_GE(out.alpha_rad, alpha_prev) << "non-monotonic α at n=" << n;
        alpha_prev = out.alpha_rad;
    }
}

// ── Lateral, T=0 analytic check ───────────────────────────────────────────────

TEST_F(AllocatorFixture, LateralT0AnalyticCheck) {
    // With T=0, g(β) = q·S·C_Yβ·β − n_y·m·g = 0  →  β = n_y·m·g / (q·S·C_Yβ).
    const float n_y            = 0.2f; // 0.2 g lateral
    const float beta_expected  = n_y * kMass * kG / (kQ * kS * kCYb);

    LoadFactorInputs in{1.0f, n_y, kQ, 0.f, kMass};
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.beta_rad, beta_expected, 1e-4f);
}

// ── alphaDot: analytical via implicit function theorem ────────────────────────

TEST_F(AllocatorFixture, AlphaDotZeroForConstantN) {
    // n_dot=0 → alphaDot=0 at any operating point.
    LoadFactorInputs in{1.f, 0.f, kQ, 0.f, kMass};
    // n_dot defaults to 0
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.alphaDot_rps, 0.f, 1e-6f);
}

TEST_F(AllocatorFixture, AlphaDotAnalyticLinearRegionT0) {
    // T=0, linear region: f'(α) = q·S·C_Lα  →  alphaDot = m·g·n_dot / (q·S·C_Lα).
    const float n_dot             = 0.2f;
    const float alphaDot_expected = kMass * kG * n_dot
                                    / (kQ * kS * gaLiftParams().cl_alpha);
    LoadFactorInputs in{1.f, 0.f, kQ, 0.f, kMass};
    in.n_dot = n_dot;
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.alphaDot_rps, alphaDot_expected, 1e-5f);
}

TEST_F(AllocatorFixture, AlphaDotZeroAtStall) {
    // Once α is clamped to the stall peak, further load-factor demand cannot
    // increase α — alphaDot should be zero.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);
    LoadFactorInputs in{n_ceiling * 1.2f, 0.f, kQ, 0.f, kMass};
    in.n_dot = 1.f; // non-zero demand rate
    auto out  = alloc.solve(in);
    EXPECT_EQ(out.alpha_segment, LiftCurveSegment::IncipientStallPositive);
    EXPECT_NEAR(out.alphaDot_rps, 0.f, 1e-6f);
}

// ── betaDot: analytical via implicit function theorem ─────────────────────────

TEST_F(AllocatorFixture, BetaDotZeroForConstantNy) {
    // n_y_dot=0 → betaDot=0.
    LoadFactorInputs in{1.f, 0.2f, kQ, 0.f, kMass};
    // n_y_dot defaults to 0
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.betaDot_rps, 0.f, 1e-6f);
}

TEST_F(AllocatorFixture, BetaDotAnalyticT0) {
    // T=0: g'(β) = q·S·C_Yβ, dg/dα term = 0  →  betaDot = m·g·n_y_dot / (q·S·C_Yβ).
    const float n_y_dot           = 0.1f;
    const float betaDot_expected  = kMass * kG * n_y_dot / (kQ * kS * kCYb);
    LoadFactorInputs in{1.f, 0.f, kQ, 0.f, kMass};
    in.n_y_dot = n_y_dot;
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.betaDot_rps, betaDot_expected, 1e-5f);
}
