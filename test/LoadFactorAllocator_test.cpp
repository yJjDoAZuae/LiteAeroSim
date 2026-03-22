#define _USE_MATH_DEFINES
#include "aerodynamics/LoadFactorAllocator.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <nlohmann/json.hpp>
#include <vector>
#include <cstdint>

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
    // n_z_dot=0 → alphaDot=0 at any operating point.
    LoadFactorInputs in{1.f, 0.f, kQ, 0.f, kMass};
    // n_z_dot defaults to 0
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.alphaDot_rps, 0.f, 1e-6f);
}

TEST_F(AllocatorFixture, AlphaDotAnalyticLinearRegionT0) {
    // T=0, linear region: f'(α) = q·S·C_Lα  →  alphaDot = m·g·n_z_dot / (q·S·C_Lα).
    const float n_z_dot           = 0.2f;
    const float alphaDot_expected = kMass * kG * n_z_dot
                                    / (kQ * kS * gaLiftParams().cl_alpha);
    LoadFactorInputs in{1.f, 0.f, kQ, 0.f, kMass};
    in.n_z_dot = n_z_dot;
    auto out = alloc.solve(in);
    EXPECT_NEAR(out.alphaDot_rps, alphaDot_expected, 1e-5f);
}

TEST_F(AllocatorFixture, AlphaDotZeroAtStall) {
    // Once α is clamped to the stall peak, further load-factor demand cannot
    // increase α — alphaDot should be zero.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);
    LoadFactorInputs in{n_ceiling * 1.2f, 0.f, kQ, 0.f, kMass};
    in.n_z_dot = 1.f; // non-zero demand rate
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

// ── Serialization ─────────────────────────────────────────────────────────────

// Advance the allocator with several solve() calls to build warm-start state.
static void warmUpAllocator(LoadFactorAllocator& alloc) {
    alloc.solve({1.0f, 0.1f, kQ, 0.f, kMass});
    alloc.solve({1.5f, 0.2f, kQ, 0.f, kMass});
    alloc.solve({2.0f, 0.0f, kQ, 0.f, kMass});
}

TEST_F(AllocatorFixture, JsonRoundTrip) {
    warmUpAllocator(alloc);
    const nlohmann::json j = alloc.serializeJson();

    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    restored.deserializeJson(j);

    // Both allocators should produce identical output on the next solve().
    LoadFactorInputs in{1.2f, 0.15f, kQ, 0.f, kMass};
    const LoadFactorOutputs outOriginal = alloc.solve(in);
    const LoadFactorOutputs outRestored = restored.solve(in);
    EXPECT_NEAR(outRestored.alpha_rad, outOriginal.alpha_rad, 1e-5f);
    EXPECT_NEAR(outRestored.beta_rad,  outOriginal.beta_rad,  1e-5f);
}

TEST_F(AllocatorFixture, JsonSchemaVersionMismatchThrows) {
    nlohmann::json j = alloc.serializeJson();
    j["schema_version"] = 99;
    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    EXPECT_THROW(restored.deserializeJson(j), std::runtime_error);
}

TEST_F(AllocatorFixture, ProtoRoundTrip) {
    warmUpAllocator(alloc);
    const std::vector<uint8_t> bytes = alloc.serializeProto();

    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    restored.deserializeProto(bytes);

    LoadFactorInputs in{1.2f, 0.15f, kQ, 0.f, kMass};
    const LoadFactorOutputs outOriginal = alloc.solve(in);
    const LoadFactorOutputs outRestored = restored.solve(in);
    EXPECT_NEAR(outRestored.alpha_rad, outOriginal.alpha_rad, 1e-4f);
    EXPECT_NEAR(outRestored.beta_rad,  outOriginal.beta_rad,  1e-4f);
}

// ── Positive-thrust α* ceiling ────────────────────────────────────────────────
//
// With positive thrust the maximum achievable Nz occurs at α* > alpha_peak,
// where f'(α) = qS·CL'(α) + T·cos(α) = 0 (the derivative zero-crossing).
// The solver must:
//   (a) converge to solutions at α > alpha_peak when Nz is in (Nz_peak, Nz_star], and
//   (b) clamp at α* (not alpha_peak) when Nz exceeds the ceiling.
//
// Helper: bisect f'(α) = 0 in [lo, hi] where f'(lo) > 0 and f'(hi) < 0.
static float bisectFprimeCrossing(const LiftCurveModel& lift,
                                   float qS, float T,
                                   float lo, float hi) {
    for (int i = 0; i < 60; ++i) {
        const float mid = 0.5f * (lo + hi);
        if (qS * lift.derivative(mid) + T * std::cos(mid) > 0.f) lo = mid;
        else                                                        hi = mid;
    }
    return 0.5f * (lo + hi);
}

// Achievable Nz at a given α.
static float achievableNz(const LiftCurveModel& lift,
                           float alpha, float qS, float T, float mg) {
    return (qS * lift.evaluate(alpha) + T * std::sin(alpha)) / mg;
}

// Large positive thrust — shifts α* visibly above alpha_peak.
static constexpr float kLargeThrust = 50000.f; // N

// hi bound for bisectFprimeCrossing: 0.07 rad above alpha_peak puts us well
// inside the post-stall quadratic (below alpha_sep ≈ 0.318 rad for these params)
// where CL' < 0 and f'(hi) < 0 for kLargeThrust.
static float bisectHiBound(const LiftCurveModel& lift) {
    return lift.alphaPeak() + 0.07f;
}

TEST_F(AllocatorFixture, WithThrust_SolutionExistsAboveAlphaPeak) {
    // A commanded Nz between Nz(alpha_peak, T) and Nz(alpha*, T) has a valid
    // solution at α > alpha_peak.  The solver must not clamp prematurely.
    const float qS = kQ * kS;
    const float mg = kMass * kG;

    const float alpha_star = bisectFprimeCrossing(lift, qS, kLargeThrust,
                                                   lift.alphaPeak(),
                                                   bisectHiBound(lift));
    const float nz_at_peak = achievableNz(lift, lift.alphaPeak(), qS, kLargeThrust, mg);
    const float nz_at_star = achievableNz(lift, alpha_star,       qS, kLargeThrust, mg);

    ASSERT_GT(alpha_star, lift.alphaPeak()) << "test setup: α* must be above alpha_peak";
    ASSERT_GT(nz_at_star, nz_at_peak)       << "test setup: ceiling must exceed Nz at alpha_peak";

    // Command Nz midway between Nz(alpha_peak) and Nz(alpha*); solution at α > alpha_peak.
    const float n_cmd = 0.5f * (nz_at_peak + nz_at_star);

    // Warm-start from just below to help Newton approach from the right side.
    alloc.solve({n_cmd * 0.9f, 0.f, kQ, kLargeThrust, kMass});
    const auto out = alloc.solve({n_cmd, 0.f, kQ, kLargeThrust, kMass});

    // Equation residual must be near zero (good convergence).
    const float residual = qS * lift.evaluate(out.alpha_rad)
                           + kLargeThrust * std::sin(out.alpha_rad)
                           - n_cmd * mg;
    EXPECT_NEAR(residual, 0.f, 10.f)
        << "f(α) should be satisfied; premature clamp at alpha_peak leaves residual";

    // α must have advanced past alpha_peak toward the true solution.
    EXPECT_GT(out.alpha_rad, lift.alphaPeak())
        << "solver clamped prematurely at alpha_peak with positive thrust";
}

TEST_F(AllocatorFixture, WithThrust_ExcessNzClampsAtFprimeCrossing) {
    // When Nz is above the thrust-augmented ceiling, α must clamp at α*
    // (the f'=0 crossing), not at alpha_peak.
    const float qS = kQ * kS;

    const float alpha_star = bisectFprimeCrossing(lift, qS, kLargeThrust,
                                                   lift.alphaPeak(),
                                                   bisectHiBound(lift));

    const auto out = alloc.solve({20.f, 0.f, kQ, kLargeThrust, kMass});

    EXPECT_GT(out.alpha_rad, lift.alphaPeak())
        << "excess Nz with thrust must clamp above alpha_peak";
    EXPECT_NEAR(out.alpha_rad, alpha_star, 1e-3f)
        << "excess Nz with thrust must clamp at α* (f'=0), not alpha_peak";
}

TEST_F(AllocatorFixture, ZeroThrust_ExcessNzStillClampsAtAlphaPeak) {
    // With T=0, α* == alpha_peak (T·cos term vanishes). Existing behaviour must
    // be preserved: clamping at alpha_peak is correct.
    const auto out = alloc.solve({20.f, 0.f, kQ, 0.f, kMass});
    EXPECT_NEAR(out.alpha_rad, lift.alphaPeak(), 1e-4f);
}

TEST_F(AllocatorFixture, WithThrust_AlphaMonotonicAndReachesAlphaStar) {
    // Sweep Nz from 1 g through and past the thrust-augmented ceiling.
    // α must increase monotonically throughout and must reach α* (not stall
    // at alpha_peak) once Nz exceeds the ceiling.
    const float qS         = kQ * kS;
    const float mg         = kMass * kG;
    const float alpha_star = bisectFprimeCrossing(lift, qS, kLargeThrust,
                                                   lift.alphaPeak(),
                                                   bisectHiBound(lift));
    const float nz_ceiling = achievableNz(lift, alpha_star, qS, kLargeThrust, mg);

    float alpha_prev  = 0.f;
    float alpha_final = 0.f;
    for (int i = 0; i <= 100; ++i) {
        const float t   = static_cast<float>(i) / 100.f;
        const float n_z = 1.f + t * (nz_ceiling * 1.5f - 1.f);
        const auto  out = alloc.solve({n_z, 0.f, kQ, kLargeThrust, kMass});

        EXPECT_GE(out.alpha_rad, alpha_prev - 1e-4f)
            << "non-monotonic α at step " << i << " (nz=" << n_z << ")";
        EXPECT_LE(out.alpha_rad, alpha_star + 1e-3f)
            << "α exceeded α* at step " << i;
        alpha_prev  = out.alpha_rad;
        alpha_final = out.alpha_rad;
    }
    // At 1.5× ceiling the solver must be clamped at α*, not stuck at alpha_peak.
    EXPECT_NEAR(alpha_final, alpha_star, 1e-3f)
        << "α must reach α* when Nz is above the thrust-augmented ceiling";
}

TEST_F(AllocatorFixture, ProtoSchemaVersionMismatchThrows) {
    const std::vector<uint8_t> bytes = alloc.serializeProto();
    // Corrupt: modify schema_version field (field 1, varint, tag = 0x08) to value 99.
    std::vector<uint8_t> bad = bytes;
    for (std::size_t i = 0; i + 1 < bad.size(); ++i) {
        if (bad[i] == 0x08) {
            bad[i + 1] = 99;
            break;
        }
    }
    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    EXPECT_THROW(restored.deserializeProto(bad), std::runtime_error);
}
