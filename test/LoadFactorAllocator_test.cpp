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

// ── Branch-continuation predictor ─────────────────────────────────────────────
//
// The predictor computes a first-order warm-start:
//   α₀ = α_prev + δn_z · m·g / f′(α_prev)
//   β₀ = β_prev + δn_y · m·g / g′(β_prev)
//
// In the linear lift region f is linear in α, so the predictor is exact: it
// places Newton's initial iterate directly at the solution.  The solver
// converges in a single iteration (i.e. the loop runs once and then checks
// that f(α₀) = 0, yielding |α_new − α₀| = 0 < kTol).
//
// Without the predictor the warm-start is α_prev (the previous solution), so
// a linear step of Δn = 0.1 g requires two loop iterations: one large Newton
// step to the solution, then a second iteration to confirm |Δ| < kTol.
//
// The three tests below will not compile until `iterations` is added to
// LoadFactorOutputs, and the convergence test will fail at runtime until the
// predictor is implemented.

TEST_F(AllocatorFixture, PredictorReducesIterationsOnLinearStep) {
    // Establish warm-start at n=1 g (linear region, α ≈ 0.0699 rad).
    alloc.solve({1.0f, 0.f, kQ, 0.f, kMass});

    // Small step to n=1.1 g — still fully in the linear region.
    // With the predictor, α₀ = α_prev + 0.1·m·g / (q·S·C_Lα) which is the
    // exact solution: f(α₀) = 0 → |α_new − α₀| = 0 → converged on iteration 1.
    const auto out = alloc.solve({1.1f, 0.f, kQ, 0.f, kMass});

    // Correct solution sanity check.
    const float alpha_expected = 1.1f * kMass * kG / (kQ * kS * gaLiftParams().cl_alpha);
    EXPECT_NEAR(out.alpha_rad, alpha_expected, 1e-5f);

    // With predictor: 1 iteration.  Without predictor: 2 iterations.
    EXPECT_EQ(out.iterations, 1)
        << "predictor must place the warm-start at the exact solution in the "
           "linear region, converging in 1 iteration";
}

TEST_F(AllocatorFixture, PredictorJsonRoundTrip_IncludesNzPrevAndNyPrev) {
    // After a solve the allocator must persist n_z_prev and n_y_prev so that
    // the branch-continuation predictor state survives serialization.
    alloc.solve({1.5f, 0.1f, kQ, 0.f, kMass});
    const nlohmann::json j = alloc.serializeJson();

    ASSERT_TRUE(j.contains("n_z_prev_nd")) << "JSON missing n_z_prev_nd";
    ASSERT_TRUE(j.contains("n_y_prev_nd")) << "JSON missing n_y_prev_nd";
    EXPECT_NEAR(j["n_z_prev_nd"].get<float>(), 1.5f, 1e-4f);
    EXPECT_NEAR(j["n_y_prev_nd"].get<float>(), 0.1f, 1e-4f);

    // After round-trip, a subsequent linear step must also converge in 1 iteration.
    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    restored.deserializeJson(j);

    const auto outOrig = alloc.solve({1.6f, 0.1f, kQ, 0.f, kMass});
    const auto outRest = restored.solve({1.6f, 0.1f, kQ, 0.f, kMass});

    EXPECT_NEAR(outRest.alpha_rad, outOrig.alpha_rad, 1e-5f);
    EXPECT_EQ(outRest.iterations, 1)
        << "restored allocator must predict correctly — n_z_prev must be serialized";
}

TEST_F(AllocatorFixture, PredictorProtoRoundTrip_IncludesNzPrev) {
    // Proto serialization must also preserve n_z_prev / n_y_prev.
    alloc.solve({1.5f, 0.1f, kQ, 0.f, kMass});
    const std::vector<uint8_t> bytes = alloc.serializeProto();

    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb);
    restored.deserializeProto(bytes);

    const auto outOrig = alloc.solve({1.6f, 0.1f, kQ, 0.f, kMass});
    const auto outRest = restored.solve({1.6f, 0.1f, kQ, 0.f, kMass});

    EXPECT_NEAR(outRest.alpha_rad, outOrig.alpha_rad, 1e-5f);
    EXPECT_EQ(outRest.iterations, 1)
        << "restored allocator (proto) must predict correctly — n_z_prev must be serialized";
}

// ── Continuity and monotonicity across all lift-curve segments ─────────────────
//
// Derived values for gaLiftParams() (cl_alpha=5.73, cl_max=1.20, delta=0.05):
//   alpha_peak = cl_max/cl_alpha + delta/2  ≈  0.234 rad
//   alpha_star = alpha_peak − delta         ≈  0.184 rad   (Linear↔Incipient join)
//   n_ceiling  = cl_max·qS/mg              ≈  3.0 g
//   n_star     = cl_alpha·alpha_star·qS/mg ≈  2.64 g       (n at the join)
//   Symmetric negative values by construction.

TEST_F(AllocatorFixture, MonotonicNz_PositiveSweepThroughStall) {
    // Sweeps n from 0 to 120 % of the positive stall ceiling, covering the full
    // path: Linear → IncipientStall → clamp at alphaPeak.
    // Extends MonotonicNStaysPreStall (which stops at 95 %) to include the
    // incipient-stall transition and the clamp itself.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);

    float alpha_prev = 0.f;
    LoadFactorOutputs last{};
    for (int i = 1; i <= 120; ++i) {
        const float n = n_ceiling * (static_cast<float>(i) / 100.f);
        last = alloc.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_GE(last.alpha_rad, alpha_prev - 1e-4f)
            << "non-monotonic α at n = " << n << " g (step " << i << ")";
        alpha_prev = last.alpha_rad;
    }
    EXPECT_NEAR(last.alpha_rad, lift.alphaPeak(), 1e-4f)
        << "α must clamp at alphaPeak when Nz exceeds the stall ceiling";
}

TEST_F(AllocatorFixture, MonotonicNz_NegativeSweepThroughStall) {
    // Sweeps n from 0 to 120 % of the negative stall ceiling, covering the full
    // negative path: Linear → IncipientStallNegative → clamp at alphaTrough.
    const float n_ceiling_neg = gaLiftParams().cl_min * kQ * kS / (kMass * kG);

    float alpha_prev = 0.f;
    LoadFactorOutputs last{};
    for (int i = 1; i <= 120; ++i) {
        const float n = n_ceiling_neg * (static_cast<float>(i) / 100.f);
        last = alloc.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_LE(last.alpha_rad, alpha_prev + 1e-4f)
            << "non-monotonic α at n = " << n << " g (step " << i << ")";
        alpha_prev = last.alpha_rad;
    }
    EXPECT_NEAR(last.alpha_rad, lift.alphaTrough(), 1e-4f)
        << "α must clamp at alphaTrough when Nz is below the negative stall ceiling";
}

TEST_F(AllocatorFixture, SegmentBoundary_AlphaMonotonicAcrossLinearIncipientJoin) {
    // Fine-step (0.02 g) sweep from 2.5 g through the Linear→IncipientStall
    // join (n_star ≈ 2.64 g) to 2.85 g.  The C¹ join in C_L(α) implies a
    // continuous dα/dn — no jump in α should occur at the segment boundary.
    // Both the Linear and IncipientStallPositive segments must be observed.
    constexpr float kDn = 0.02f;

    // Establish warm-start just below the sweep range.
    alloc.solve({2.4f, 0.f, kQ, 0.f, kMass});
    float alpha_prev  = alloc.solve({2.5f, 0.f, kQ, 0.f, kMass}).alpha_rad;

    bool saw_linear    = false;
    bool saw_incipient = false;

    for (float n = 2.5f + kDn; n <= 2.85f + 1e-4f; n += kDn) {
        const auto out = alloc.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_GE(out.alpha_rad, alpha_prev - 1e-4f)
            << "non-monotonic α near the Linear→IncipientStall join at n = " << n;
        if (out.alpha_segment == LiftCurveSegment::Linear)                  saw_linear    = true;
        if (out.alpha_segment == LiftCurveSegment::IncipientStallPositive)  saw_incipient = true;
        alpha_prev = out.alpha_rad;
    }
    EXPECT_TRUE(saw_linear)    << "sweep must pass through the Linear segment";
    EXPECT_TRUE(saw_incipient) << "sweep must reach the IncipientStallPositive segment";
}

TEST_F(AllocatorFixture, StallRecovery_RequiresReset) {
    // After a stall call (_alpha_prev = alphaPeak), Newton starts at the fold
    // point where f'(alphaPeak) ≈ 0.  In floating-point arithmetic the residual
    // f'(alphaPeak) is nonzero but tiny, so the fold guard (kTol = 1e-6) does
    // not fire; Newton then takes a huge step to the wrong branch.
    //
    // Calling reset() clears the warm-start to zero.  The branch-continuation
    // predictor then places Newton directly at the correct linear-region
    // solution, converging in one iteration.
    //
    // This documents the correct usage: call reset() after any discontinuous
    // change in demand (e.g., re-engagement after a stall event).
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);
    const float n_1g      = 1.0f;
    const float alpha_1g  = n_1g * kMass * kG / (kQ * kS * gaLiftParams().cl_alpha);

    // Drive to stall; warm-start is now at alphaPeak.
    alloc.solve({n_ceiling * 1.5f, 0.f, kQ, 0.f, kMass});

    // Without reset(): stale warm-start at the fold point causes the solver to
    // converge to the wrong branch — result is far from the correct 1 g value.
    const auto out_wrong = alloc.solve({n_1g, 0.f, kQ, 0.f, kMass});
    EXPECT_GT(std::abs(out_wrong.alpha_rad - alpha_1g), 0.1f)
        << "without reset(), stale warm-start at alphaPeak must corrupt the solve";

    // After reset(): predictor finds the exact linear solution in one iteration.
    alloc.reset();
    const auto out_correct = alloc.solve({n_1g, 0.f, kQ, 0.f, kMass});
    EXPECT_EQ(out_correct.alpha_segment, LiftCurveSegment::Linear)
        << "after reset(), solver must return to the Linear segment";
    EXPECT_NEAR(out_correct.alpha_rad, alpha_1g, 1e-4f)
        << "after reset(), α must equal the linear-region solution";
}

// ── Black-box continuity: α must be monotone in Nz across the full domain ─────
//
// These tests treat the allocator as a black box.  They reference only the
// test-fixture sizing parameters (kMass, kS, kQ, kCYb, kLargeThrust) — never
// internal lift-curve quantities such as alphaPeak, alphaStar, or n_ceiling.
//
// Physical basis: for fixed T the normal-force equation f(α,n)=0 defines a
// unique monotone branch α(n).  A positive increment in Nz must produce a
// non-negative increment in α everywhere in the domain, including through the
// stall transition and the clamped plateau.  10 g safely exceeds the stall
// ceiling for both the zero-thrust and large-thrust cases.

TEST_F(AllocatorFixture, Alpha_IsMonotone_ZeroThrust_PositiveSweep) {
    // Sweeps Nz from 0 to +10 g in 0.02 g steps with warm-starting (T = 0).
    // Covers the pre-stall, stall-transition, and clamped-plateau regions.
    float alpha_prev = 0.f;
    for (int i = 1; i <= 500; ++i) {
        const float n   = static_cast<float>(i) * 0.02f;   // 0.02 g … 10 g
        const auto  out = alloc.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_GE(out.alpha_rad, alpha_prev - 1e-4f)
            << "α decreased on positive sweep at n = " << n << " g (T = 0)";
        alpha_prev = out.alpha_rad;
    }
}

TEST_F(AllocatorFixture, Alpha_IsMonotone_ZeroThrust_NegativeSweep) {
    // Sweeps Nz from 0 to −10 g in 0.02 g steps with warm-starting (T = 0).
    // Covers the negative pre-stall, stall-transition, and clamped-plateau regions.
    float alpha_prev = 0.f;
    for (int i = 1; i <= 500; ++i) {
        const float n   = static_cast<float>(-i) * 0.02f;  // −0.02 g … −10 g
        const auto  out = alloc.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_LE(out.alpha_rad, alpha_prev + 1e-4f)
            << "α increased on negative sweep at n = " << n << " g (T = 0)";
        alpha_prev = out.alpha_rad;
    }
}

TEST_F(AllocatorFixture, Alpha_IsMonotone_WithThrust_PositiveSweep) {
    // Sweeps Nz from 0 to +10 g with T = kLargeThrust.  Positive thrust raises
    // the stall ceiling above the zero-thrust alphaPeak; the test verifies that
    // α is monotone through this thrust-augmented ceiling without referencing
    // its location.
    float alpha_prev = 0.f;
    for (int i = 1; i <= 500; ++i) {
        const float n   = static_cast<float>(i) * 0.02f;
        const auto  out = alloc.solve({n, 0.f, kQ, kLargeThrust, kMass});
        EXPECT_GE(out.alpha_rad, alpha_prev - 1e-4f)
            << "α decreased on positive sweep at n = " << n << " g (T = kLargeThrust)";
        alpha_prev = out.alpha_rad;
    }
}

// ── Alpha limit box constraint ────────────────────────────────────────────────
//
// alpha_max_rad / alpha_min_rad are passed to the constructor and enforced at
// every Newton iterate (project-then-check).
//
// Sizing for these tests:
//   Linear-region solution at n=1 g: α ≈ 0.0699 rad  (< alpha_star ≈ 0.184 rad)
//   alpha_max constraint = 0.10 rad  → binds before the stall fold
//   alpha_min constraint = -0.10 rad → binds before the negative stall fold

// Helper: allocator with symmetric alpha limits at ±0.10 rad.
static constexpr float kAlphaMax =  0.10f; // rad — below alpha_star ≈ 0.184 rad
static constexpr float kAlphaMin = -0.10f; // rad

static LoadFactorAllocator makeConstrained(const LiftCurveModel& lift) {
    return LoadFactorAllocator(lift, kS, kCYb, kAlphaMin, kAlphaMax);
}

TEST_F(AllocatorFixture, AlphaMax_NotActiveInLinearRegion) {
    // n=1 g → α ≈ 0.0699 rad < kAlphaMax=0.10 rad: constraint is inactive.
    // Constrained and unconstrained solvers must agree.
    auto constrained = makeConstrained(lift);
    const LoadFactorInputs in{1.0f, 0.f, kQ, 0.f, kMass};
    const auto out_unc = alloc.solve(in);
    const auto out_con = constrained.solve(in);
    EXPECT_NEAR(out_con.alpha_rad, out_unc.alpha_rad, 1e-4f);
    EXPECT_LT(out_con.alpha_rad, kAlphaMax);
}

TEST_F(AllocatorFixture, AlphaMax_ClampsIterateWhenDemandExceedsLimit) {
    // Command a load factor that in the unconstrained solver gives α > kAlphaMax.
    // The constrained solver must return exactly kAlphaMax.
    // Unconstrained n=2 g → α ≈ 0.140 rad > kAlphaMax=0.10 rad.
    auto constrained = makeConstrained(lift);
    const LoadFactorInputs in{2.0f, 0.f, kQ, 0.f, kMass};
    const auto out_unc = alloc.solve(in);
    ASSERT_GT(out_unc.alpha_rad, kAlphaMax) << "test setup: unconstrained must exceed limit";
    const auto out_con = constrained.solve(in);
    EXPECT_NEAR(out_con.alpha_rad, kAlphaMax, 1e-5f);
}

TEST_F(AllocatorFixture, AlphaMin_ClampsIterateWhenDemandExceedsNegativeLimit) {
    // Command a negative load factor that gives α < kAlphaMin in the unconstrained solver.
    // n=-2 g → α ≈ -0.140 rad < kAlphaMin=-0.10 rad.
    auto constrained = makeConstrained(lift);
    const LoadFactorInputs in{-2.0f, 0.f, kQ, 0.f, kMass};
    const auto out_unc = alloc.solve(in);
    ASSERT_LT(out_unc.alpha_rad, kAlphaMin) << "test setup: unconstrained must be below limit";
    const auto out_con = constrained.solve(in);
    EXPECT_NEAR(out_con.alpha_rad, kAlphaMin, 1e-5f);
}

TEST_F(AllocatorFixture, AlphaMax_WarmStartRespectsBound) {
    // After several constrained solves the warm-start must not drift outside [alpha_min, alpha_max].
    auto constrained = makeConstrained(lift);
    // Sweep upward until constraint is active, then continue.
    for (int i = 1; i <= 30; ++i) {
        const float n = static_cast<float>(i) * 0.1f;
        const auto out = constrained.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_LE(out.alpha_rad, kAlphaMax + 1e-5f)
            << "alpha exceeded alpha_max at n=" << n;
    }
}

TEST_F(AllocatorFixture, AlphaMin_WarmStartRespectsBound) {
    auto constrained = makeConstrained(lift);
    for (int i = 1; i <= 30; ++i) {
        const float n = static_cast<float>(-i) * 0.1f;
        const auto out = constrained.solve({n, 0.f, kQ, 0.f, kMass});
        EXPECT_GE(out.alpha_rad, kAlphaMin - 1e-5f)
            << "alpha exceeded alpha_min at n=" << n;
    }
}

TEST_F(AllocatorFixture, AlphaMax_AboveAlphaPeak_T0_StaysAtAlphaPeak) {
    // T=0: f'(α) < 0 everywhere past alphaPeak, so extending alpha past the fold
    // provides no additional lift.  alpha must stay at alphaPeak even if alpha_max > alphaPeak.
    const float alpha_max = 0.30f; // rad > alphaPeak ≈ 0.234
    ASSERT_GT(alpha_max, lift.alphaPeak()) << "test setup: alpha_max must exceed alphaPeak";

    LiftCurveModel lift2(gaLiftParams());
    LoadFactorAllocator constrained(lift2, kS, kCYb, -alpha_max, alpha_max);

    const auto out = constrained.solve({10.0f, 0.f, kQ, 0.f, kMass});
    EXPECT_NEAR(out.alpha_rad, lift.alphaPeak(), 1e-4f)
        << "T=0: alpha must remain at alphaPeak even if alpha_max > alphaPeak";
}

TEST_F(AllocatorFixture, AlphaMax_AboveAlphaPeak_WithThrust_ExcessNzReachesAlphaMax) {
    // With sufficient thrust, f'(α) = qS·CL'(α) + T·cos(α) > 0 between alphaPeak
    // and alpha* (the thrust-augmented fold).  alpha_max_rad is placed in that window.
    // When Nz is unachievable, alpha must extend to alpha_max_rad, not stop at alphaPeak.
    //
    // Find alpha_max strictly between alphaPeak and the thrust-augmented alpha*:
    //   alpha* ≈ bisectFprimeCrossing(lift, qS, kLargeThrust, alphaPeak, hi)
    // Use the midpoint as alpha_max.
    const float qS  = kQ * kS;
    const float hi  = bisectHiBound(lift);
    const float alpha_star = bisectFprimeCrossing(lift, qS, kLargeThrust, lift.alphaPeak(), hi);
    const float alpha_max  = 0.5f * (lift.alphaPeak() + alpha_star);

    ASSERT_GT(alpha_max, lift.alphaPeak()) << "test setup: alpha_max must exceed alphaPeak";
    ASSERT_LT(alpha_max, alpha_star)       << "test setup: alpha_max must be below alpha*";

    // Verify f'(alpha_max) > 0 with kLargeThrust.
    const float fp_at_max = qS * lift.derivative(alpha_max) + kLargeThrust * std::cos(alpha_max);
    ASSERT_GT(fp_at_max, 0.f) << "test setup: f'(alpha_max) must be positive with kLargeThrust";

    // Nz ceiling at alpha_max is achievableNz(alpha_max) — demand well above it.
    const float mg       = kMass * kG;
    const float nz_at_max = achievableNz(lift, alpha_max, qS, kLargeThrust, mg);
    const float n_cmd    = nz_at_max * 1.5f; // 50 % above what alpha_max can provide

    LiftCurveModel lift2(gaLiftParams());
    LoadFactorAllocator constrained(lift2, kS, kCYb, -alpha_max, alpha_max);
    constrained.solve({n_cmd * 0.5f, 0.f, kQ, kLargeThrust, kMass}); // warm-start
    const auto out = constrained.solve({n_cmd, 0.f, kQ, kLargeThrust, kMass});

    EXPECT_NEAR(out.alpha_rad, alpha_max, 1e-4f)
        << "with thrust, alpha must reach alpha_max_rad when Nz is unachievable and f'(alpha_max)>0";
    EXPECT_GT(out.alpha_rad, lift.alphaPeak())
        << "alpha must exceed alphaPeak when thrust keeps f' positive at alpha_max";
}

TEST_F(AllocatorFixture, AlphaMax_WhenNzExceedsBothStallAndLimit) {
    // Scenario: alpha_max_rad is below alphaPeak, so the physical alpha limit
    // becomes binding before the aerodynamic stall fold.  When the Nz command
    // exceeds what is achievable at alpha_max (insufficient lift + thrust),
    // the solver must clamp at alpha_max — not at alphaPeak.
    //
    // alpha_max_rad = 0.10 rad < alphaPeak ≈ 0.234 rad.
    // At alpha = 0.10 rad, CL ≈ cl_alpha * 0.10 ≈ 0.573.
    // Max achievable Nz at alpha_max (T=0): cl*qS/mg ≈ 1.43 g.
    // Demand n = 3 g >> 1.43 g → limit at alpha_max_rad.
    auto constrained = makeConstrained(lift);
    const LoadFactorInputs in{3.0f, 0.f, kQ, 0.f, kMass};

    // Sanity: unconstrained solver would reach alphaPeak under excess Nz.
    const auto out_unc = alloc.solve(in);
    EXPECT_NEAR(out_unc.alpha_rad, lift.alphaPeak(), 1e-4f)
        << "unconstrained: excess Nz must clamp at alphaPeak";

    // Constrained solver must clamp at alpha_max_rad (tighter than alphaPeak).
    const auto out_con = constrained.solve(in);
    EXPECT_NEAR(out_con.alpha_rad, kAlphaMax, 1e-5f)
        << "constrained: alpha_max_rad < alphaPeak must be the binding limit";
    EXPECT_LT(out_con.alpha_rad, lift.alphaPeak())
        << "constrained alpha must not reach alphaPeak when alpha_max is tighter";
}

TEST_F(AllocatorFixture, AlphaConstraint_DoesNotAffectUnconstrained) {
    // Default constructor (no alpha limits) must behave identically to before.
    // Unconstrained solver reaches the stall fold without being clipped at kAlphaMax.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);
    const auto out = alloc.solve({n_ceiling * 1.2f, 0.f, kQ, 0.f, kMass});
    EXPECT_NEAR(out.alpha_rad, lift.alphaPeak(), 1e-4f)
        << "default (unconstrained) allocator must still clamp at alphaPeak";
}

TEST_F(AllocatorFixture, AlphaConstraint_JsonRoundTrip) {
    // Alpha limits must survive JSON serialization/deserialization.
    auto constrained = makeConstrained(lift);
    constrained.solve({2.0f, 0.f, kQ, 0.f, kMass}); // activate constraint, build warm-start
    const nlohmann::json j = constrained.serializeJson();

    ASSERT_TRUE(j.contains("alpha_max_rad")) << "JSON missing alpha_max_rad";
    ASSERT_TRUE(j.contains("alpha_min_rad")) << "JSON missing alpha_min_rad";
    EXPECT_NEAR(j["alpha_max_rad"].get<float>(), kAlphaMax, 1e-6f);
    EXPECT_NEAR(j["alpha_min_rad"].get<float>(), kAlphaMin, 1e-6f);

    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb, kAlphaMin, kAlphaMax);
    restored.deserializeJson(j);

    // Restored allocator must also clamp at kAlphaMax.
    const auto out_orig = constrained.solve({2.5f, 0.f, kQ, 0.f, kMass});
    const auto out_rest = restored.solve({2.5f, 0.f, kQ, 0.f, kMass});
    EXPECT_NEAR(out_rest.alpha_rad, kAlphaMax, 1e-5f);
    EXPECT_NEAR(out_rest.alpha_rad, out_orig.alpha_rad, 1e-5f);
}

TEST_F(AllocatorFixture, AlphaConstraint_ProtoRoundTrip) {
    // Alpha limits must survive proto serialization/deserialization.
    auto constrained = makeConstrained(lift);
    constrained.solve({2.0f, 0.f, kQ, 0.f, kMass});
    const std::vector<uint8_t> bytes = constrained.serializeProto();

    LiftCurveModel liftRestored(gaLiftParams());
    LoadFactorAllocator restored(liftRestored, kS, kCYb, kAlphaMin, kAlphaMax);
    restored.deserializeProto(bytes);

    const auto out_orig = constrained.solve({2.5f, 0.f, kQ, 0.f, kMass});
    const auto out_rest = restored.solve({2.5f, 0.f, kQ, 0.f, kMass});
    EXPECT_NEAR(out_rest.alpha_rad, kAlphaMax, 1e-5f);
    EXPECT_NEAR(out_rest.alpha_rad, out_orig.alpha_rad, 1e-4f);
}

TEST_F(AllocatorFixture, Alpha_Perturbation_IsMonotone_FullDomain) {
    // At each operating point on a uniform grid, verifies that a positive
    // perturbation δ in Nz produces a non-negative change in α.  Each point
    // uses a fresh allocator so the result is independent of the warm-start
    // path — a point-wise continuity check independent of traversal order.
    //
    // T = 0 case:    n₀ ∈ [−9 g, +9 g], step 0.5 g (37 points).
    // T > 0 case:    n₀ ∈ [  0 g, +9 g], step 0.5 g (19 points).
    //   (Large positive thrust makes very negative Nz infeasible; the positive
    //   half covers the thrust-augmented stall region without hitting that limit.)
    constexpr float kDelta = 0.05f;  // g

    LiftCurveModel scratch_lift(gaLiftParams());

    // T = 0: full signed domain.
    for (int i = -18; i <= 18; ++i) {
        const float n0 = static_cast<float>(i) * 0.5f;  // −9 g … +9 g

        LoadFactorAllocator lo(scratch_lift, kS, kCYb);
        LoadFactorAllocator hi(scratch_lift, kS, kCYb);

        const float alpha_lo = lo.solve({n0 - kDelta, 0.f, kQ, 0.f, kMass}).alpha_rad;
        const float alpha_hi = hi.solve({n0 + kDelta, 0.f, kQ, 0.f, kMass}).alpha_rad;

        EXPECT_LE(alpha_lo, alpha_hi + 1e-4f)
            << "T=0: α(n₀−δ) > α(n₀+δ) at n₀ = " << n0 << " g";
    }

    // T > 0: positive domain only.
    for (int i = 0; i <= 18; ++i) {
        const float n0 = static_cast<float>(i) * 0.5f;  // 0 g … +9 g

        LoadFactorAllocator lo(scratch_lift, kS, kCYb);
        LoadFactorAllocator hi(scratch_lift, kS, kCYb);

        const float alpha_lo = lo.solve({n0 - kDelta, 0.f, kQ, kLargeThrust, kMass}).alpha_rad;
        const float alpha_hi = hi.solve({n0 + kDelta, 0.f, kQ, kLargeThrust, kMass}).alpha_rad;

        EXPECT_LE(alpha_lo, alpha_hi + 1e-4f)
            << "T=kLargeThrust: α(n₀−δ) > α(n₀+δ) at n₀ = " << n0 << " g";
    }
}

// ── Stall dynamics — shared constants ────────────────────────────────────────

static constexpr float kAlphaDotMax = 0.5f;           // rad/s — rate limit for bridge tests
static constexpr float kDt          = 0.01f;          // s     — time step for bridge tests
static constexpr float kHalfPi      = 1.5707963f;     // π/2 (default non-binding alpha limit)

// Helper: build allocator with rate-limit enabled.
static LoadFactorAllocator makeRateLimited(const LiftCurveModel& lift) {
    return LoadFactorAllocator(lift, kS, kCYb, -kHalfPi, kHalfPi, kAlphaDotMax);
}

// Helper: inject stall state via JSON so the entry-condition sequence is not needed.
static void injectStalledState(LoadFactorAllocator& alloc, float alpha_prev_rad,
                               float cl_recovering, bool stalled) {
    nlohmann::json j = alloc.serializeJson();
    j["alpha_prev_rad"]  = alpha_prev_rad;
    j["stalled"]         = stalled;
    j["cl_recovering"]   = cl_recovering;
    alloc.deserializeJson(j);
}

// Helper: drive allocator with kLargeThrust to just above alphaPeak, then step
// below alphaPeak.  Returns the LoadFactorOutputs of the drop-below step.
// Uses large thrust so f'(alphaPeak) > 0 and Newton can converge reliably near the peak.
static LoadFactorOutputs driveToStall(LoadFactorAllocator& alloc, const LiftCurveModel& lift) {
    const float qS = kQ * kS;
    const float mg = kMass * kG;
    // Achievable Nz just above and just below alphaPeak (with large thrust).
    const float n_above = achievableNz(lift, lift.alphaPeak() + 0.005f, qS, kLargeThrust, mg);
    const float n_below = achievableNz(lift, lift.alphaPeak() - 0.010f, qS, kLargeThrust, mg);
    alloc.solve({n_above * 0.5f, 0.f, kQ, kLargeThrust, kMass, 0.f, 0.f, kDt});
    alloc.solve({n_above,        0.f, kQ, kLargeThrust, kMass, 0.f, 0.f, kDt});
    return alloc.solve({n_below,  0.f, kQ, kLargeThrust, kMass, 0.f, 0.f, kDt});
}

// ── Step 2 — alpha_dot_max_rad_s serialization ────────────────────────────────

TEST_F(AllocatorFixture, AlphaDotMaxSerialized_Json) {
    LiftCurveModel lift2(gaLiftParams());
    LoadFactorAllocator alloc2(lift2, kS, kCYb, -kHalfPi, kHalfPi, kAlphaDotMax);
    const nlohmann::json j = alloc2.serializeJson();
    ASSERT_TRUE(j.contains("alpha_dot_max_rad_s"));
    EXPECT_NEAR(j["alpha_dot_max_rad_s"].get<float>(), kAlphaDotMax, 1e-6f);
}

TEST_F(AllocatorFixture, AlphaDotMaxSerialized_Proto) {
    LiftCurveModel lift2(gaLiftParams());
    LoadFactorAllocator alloc2(lift2, kS, kCYb, -kHalfPi, kHalfPi, kAlphaDotMax);
    const std::vector<uint8_t> bytes = alloc2.serializeProto();

    LiftCurveModel lift3(gaLiftParams());
    LoadFactorAllocator restored(lift3, kS, kCYb);
    restored.deserializeProto(bytes);

    // Verify round-trip by comparing a subsequent solve output
    const LoadFactorInputs in{1.0f, 0.f, kQ, 0.f, kMass};
    EXPECT_NEAR(restored.solve(in).alpha_rad, alloc2.solve(in).alpha_rad, 1e-5f);
}

// ── Item 1 — Alpha rate bridge ────────────────────────────────────────────────

TEST_F(AllocatorFixture, AlphaBridge_Inactive_InNormalPreStall) {
    // Non-stalled operation: alpha_out must equal the Newton solution regardless
    // of alpha_dot_max.  Construct with rate limit but command a normal pre-stall Nz.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    LoadFactorInputs in{1.0f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
    const auto out   = alloc2.solve(in);
    const auto out_ref = alloc.solve(in);  // default allocator (no rate limit)

    EXPECT_FALSE(out.stalled);
    EXPECT_FALSE(out.stalled_neg);
    EXPECT_NEAR(out.alpha_rad, out_ref.alpha_rad, 1e-5f)
        << "bridge must not constrain alpha when not stalled";
}

TEST_F(AllocatorFixture, AlphaBridge_LimitsStepSizeWhileStalled) {
    // Inject stalled state with alpha_prev at alphaPeak, cl_recovering = clSep.
    // Command a Nz whose explicit stall-solve (with thrust) gives alpha_eq well below
    // alphePeak, so the bridge step is limiting rather than the solver.
    // The bridge must constrain the step to alpha_dot_max * dt.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    const float alpha_prev = lift2.alphaPeak();
    injectStalledState(alloc2, alpha_prev, lift2.clSep(), true);

    // Use kLargeThrust so the explicit solve gives alpha_eq far below alphePeak.
    const float n_sub = 0.5f * (gaLiftParams().cl_max * kQ * kS / (kMass * kG));
    LoadFactorInputs in{n_sub, 0.f, kQ, kLargeThrust, kMass, 0.f, 0.f, kDt};
    const auto out = alloc2.solve(in);

    // Bridge should have stepped by exactly alpha_dot_max * dt (alpha decreasing)
    const float expected_step = kAlphaDotMax * kDt;
    EXPECT_NEAR(std::abs(out.alpha_rad - alpha_prev), expected_step, 1e-5f)
        << "bridge must limit |alpha_out - alpha_prev| to alpha_dot_max * dt while stalled";
}

TEST_F(AllocatorFixture, AlphaBridge_ConvergesOverMultipleSteps) {
    // With stall flag active and many small steps, alpha_out must converge toward
    // alpha_eq.  After enough steps the stall flag clears (once alpha <= alphaStar).
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);

    const float n_target  = 1.0f; // 1 g — well below stall ceiling
    const int   max_steps = 200;  // at 0.5 rad/s rate and 0.01 s dt: ~4.6 rad/s of travel

    bool flag_cleared = false;
    for (int i = 0; i < max_steps; ++i) {
        LoadFactorInputs in{n_target, 0.f, kQ, kLargeThrust, kMass, 0.f, 0.f, kDt};
        const auto out = alloc2.solve(in);
        if (!out.stalled) {
            flag_cleared = true;
            break;
        }
    }
    EXPECT_TRUE(flag_cleared) << "stall flag must clear once alpha steps below alphaStar";
}

TEST_F(AllocatorFixture, AlphaBridge_NRealizedFromAlphaOut) {
    // Realized Nz must use cl_eff (from alpha_out), not the commanded n_z.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);

    const float n_cmd = 0.5f;
    LoadFactorInputs in{n_cmd, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
    const auto out = alloc2.solve(in);

    // Verify n_z_realized == (qS * cl_eff + T * sin(alpha_out)) / mg
    const float mg = kMass * kG;
    const float qS = kQ * kS;
    const float n_expected = (qS * out.cl_eff + 0.f * std::sin(out.alpha_rad)) / mg;
    EXPECT_NEAR(out.n_z_realized, n_expected, 1e-5f);
    // And it must differ from the commanded n (we're stalled and rate-limited)
    EXPECT_NE(out.n_z_realized, n_cmd);
}

TEST_F(AllocatorFixture, AlphaDotMax_JsonRoundTrip_PreservesRateLimit) {
    // After JSON serialization/deserialization the rate limit must survive.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);
    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);

    const nlohmann::json j = alloc2.serializeJson();
    ASSERT_TRUE(j.contains("alpha_dot_max_rad_s"));
    ASSERT_TRUE(j.contains("stalled"));
    EXPECT_TRUE(j["stalled"].get<bool>());

    LiftCurveModel lift3(gaLiftParams());
    LoadFactorAllocator restored(lift3, kS, kCYb);
    restored.deserializeJson(j);

    LoadFactorInputs in{0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
    const auto out_orig = alloc2.solve(in);
    const auto out_rest = restored.solve(in);
    EXPECT_NEAR(out_rest.alpha_rad, out_orig.alpha_rad, 1e-5f);
    EXPECT_EQ(out_rest.stalled, out_orig.stalled);
}

TEST_F(AllocatorFixture, AlphaDotMax_ProtoRoundTrip_PreservesRateLimit) {
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);
    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);

    const std::vector<uint8_t> bytes = alloc2.serializeProto();

    LiftCurveModel lift3(gaLiftParams());
    LoadFactorAllocator restored(lift3, kS, kCYb);
    restored.deserializeProto(bytes);

    LoadFactorInputs in{0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
    const auto out_orig = alloc2.solve(in);
    const auto out_rest = restored.solve(in);
    EXPECT_NEAR(out_rest.alpha_rad, out_orig.alpha_rad, 1e-4f);
    EXPECT_EQ(out_rest.stalled, out_orig.stalled);
}

// ── Item 2 — Stall hysteresis flags ──────────────────────────────────────────

TEST_F(AllocatorFixture, Hysteresis_FlagSetsWhenDecreasingFromAlphaPeak) {
    // Use large thrust so f'(alphaPeak) > 0 and Newton can step smoothly above/below peak.
    LiftCurveModel lift2(gaLiftParams());
    LoadFactorAllocator alloc2(lift2, kS, kCYb, -kHalfPi, kHalfPi, kAlphaDotMax);

    const auto out = driveToStall(alloc2, lift2);

    EXPECT_LT(out.alpha_rad, lift2.alphaPeak()) << "alpha_out must be below alphaPeak after drop";
    EXPECT_TRUE(out.stalled) << "stall flag must set when alpha decreases from alphaPeak";
}

TEST_F(AllocatorFixture, Hysteresis_FlagDoesNotSetOnAscent) {
    // Monotonically increasing Nz — alpha only ascends; stall flag must never set.
    const float n_ceiling = gaLiftParams().cl_max * kQ * kS / (kMass * kG);
    LoadFactorOutputs last{};
    for (int i = 1; i <= 120; ++i) {
        const float n = n_ceiling * (static_cast<float>(i) / 100.f);
        last = alloc.solve({n, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt});
        EXPECT_FALSE(last.stalled)    << "stall flag must not set during ascent at n=" << n;
        EXPECT_FALSE(last.stalled_neg) << "negative stall flag must not set during ascent";
    }
}

TEST_F(AllocatorFixture, Hysteresis_FlagClearsAtAlphaStar_TypicalConfig) {
    // Standard config: clSep = 0.80 < clAlpha * alphaStar ≈ 1.055
    // → threshold exit: flag clears when alpha crosses alphaStar.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    // Inject stall at alphaStar + 0.01 (still above alpheStar, flag should stay set)
    const float alpha_above_star = lift2.alphaStar() + 0.01f;
    injectStalledState(alloc2, alpha_above_star, lift2.clSep(), true);
    {
        LoadFactorInputs in{0.8f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
        const auto out = alloc2.solve(in);
        EXPECT_TRUE(out.stalled) << "flag must remain set above alphaStar";
    }

    // Now inject at alphaStar - 0.005 — flag should clear on first step
    injectStalledState(alloc2, lift2.alphaStar() - 0.005f, lift2.clSep(), true);
    {
        LoadFactorInputs in{0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
        const auto out = alloc2.solve(in);
        EXPECT_FALSE(out.stalled) << "flag must clear when alpha <= alphaStar";
    }
}

TEST_F(AllocatorFixture, Hysteresis_FlagClearsAtSnapDown_HighPlateauConfig) {
    // High-plateau config: clSep > clAlpha * alphaStar so the nominal CL descends
    // below clSep before reaching alphaStar.  Stall flag must clear at the
    // intersection above alphaStar, not at alphaStar itself.
    //
    // To achieve clSep > clAlpha * alphaStar:
    //   Use a very narrow stall break (small delta_alpha_stall) so alpheStar is close
    //   to alphePeak, and set clSep high relative to cl_alpha * alphaStar.
    //   With cl_alpha=5.73, delta=0.005, cl_max=1.20:
    //     alphaPeak ≈ 1.20/5.73 + 0.0025 ≈ 0.2120 rad
    //     alphaStar ≈ 0.2095 rad
    //     cl_alpha * alphaStar ≈ 5.73 * 0.2095 ≈ 1.200
    //   Set clSep = 1.10 < 1.20 = cl_max (required), and check:
    //     1.10 > 1.200? No.
    //   Try cl_max=1.5, delta=0.02:
    //     alphaPeak = 1.5/5.73 + 0.01 = 0.2719 rad
    //     alphaStar = 0.2519 rad
    //     cl_alpha * alphaStar = 5.73 * 0.2519 = 1.443
    //     clSep = 1.45 < 1.5 ✓ and 1.45 > 1.443 ✓  → high-plateau config
    LiftCurveParams hp;
    hp.cl_alpha              = 5.73f;
    hp.cl_max                = 1.50f;
    hp.cl_min                = -1.50f;
    hp.delta_alpha_stall     = 0.02f;
    hp.delta_alpha_stall_neg = 0.02f;
    hp.cl_sep                = 1.45f;
    hp.cl_sep_neg            = -1.45f;

    LiftCurveModel liftHP(hp);
    LoadFactorAllocator allocHP(liftHP, kS, kCYb, -kHalfPi, kHalfPi, kAlphaDotMax);

    // Verify this is indeed a high-plateau config
    ASSERT_GT(liftHP.clSep(), liftHP.clAlpha() * liftHP.alphaStar())
        << "test setup: must be high-plateau config";

    // Inject stall between alphaStar and alphaPeak (descending quadratic)
    // at a point where evaluate(alpha) < clSep (nominal is below plateau)
    // We need to find alpha_snap where evaluate(alpha) == clSep.
    // Since clSep > cl_alpha * alphaStar, the descending quadratic at alphaStar
    // has CL = cl_alpha * alphaStar < clSep, so the snap-down happens just above alphaStar.
    // Use alpha = alphaStar + 0.001 and verify evaluate(alpha) < clSep there.
    const float alpha_test = liftHP.alphaStar() + 0.001f;
    ASSERT_LT(liftHP.evaluate(alpha_test), liftHP.clSep())
        << "test setup: nominal CL must be below clSep at alpha_test for snap-down to fire";

    injectStalledState(allocHP, alpha_test, liftHP.clSep(), true);
    LoadFactorInputs in{0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
    const auto out = allocHP.solve(in);

    EXPECT_FALSE(out.stalled)
        << "flag must clear via snap-down when nominal CL descends below clSep above alphaStar";
    EXPECT_GT(out.alpha_rad, liftHP.alphaStar() - 0.05f)
        << "alpha should still be near alphaStar region, not far below";
}

TEST_F(AllocatorFixture, Hysteresis_AlphaBounceDoesNotClearFlag) {
    // Inject stall, take one step down, then command Nz that pushes alpha back up
    // partway (not reaching alphaStar). Flag must remain set.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);

    // Step down slightly
    alloc2.solve({0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt});

    // Now command high Nz that would push alpha back up (above alphaStar, still stalled)
    const float n_mid = gaLiftParams().cl_sep * kQ * kS / (kMass * kG) * 1.1f;
    LoadFactorInputs in{n_mid, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
    const auto out = alloc2.solve(in);

    EXPECT_TRUE(out.stalled) << "stall flag must not clear when alpha bounces up without reaching alphaStar";
}

TEST_F(AllocatorFixture, Hysteresis_NoFlagWithoutStallEntry) {
    // Pure pre-stall operation: flags must never be set.
    for (float n = 0.f; n <= 2.0f; n += 0.2f) {
        LoadFactorInputs in{n, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
        const auto out = alloc.solve(in);
        EXPECT_FALSE(out.stalled)     << "stall flag set spuriously at n=" << n;
        EXPECT_FALSE(out.stalled_neg) << "negative stall flag set spuriously at n=" << n;
    }
}

TEST_F(AllocatorFixture, Hysteresis_NegativeSideSymmetric) {
    // Negative stall: drive alpha to alphaTrough via large negative Nz (with thrust),
    // then step above alphaTrough. stalled_neg must set.
    LiftCurveModel lift2(gaLiftParams());
    LoadFactorAllocator alloc2(lift2, kS, kCYb, -kHalfPi, kHalfPi, kAlphaDotMax);

    const float qS = kQ * kS;
    const float mg = kMass * kG;
    const float n_above_trough = achievableNz(lift2, lift2.alphaTrough() - 0.005f, qS, kLargeThrust, mg);
    const float n_below_trough = achievableNz(lift2, lift2.alphaTrough() + 0.010f, qS, kLargeThrust, mg);

    alloc2.solve({n_above_trough * 0.5f, 0.f, kQ, kLargeThrust, kMass, 0.f, 0.f, kDt});
    alloc2.solve({n_above_trough,        0.f, kQ, kLargeThrust, kMass, 0.f, 0.f, kDt});
    const auto out = alloc2.solve({n_below_trough, 0.f, kQ, kLargeThrust, kMass, 0.f, 0.f, kDt});

    EXPECT_GT(out.alpha_rad, lift2.alphaTrough()) << "alpha must be above alphaTrough after positive step";
    EXPECT_TRUE(out.stalled_neg) << "stalled_neg must set when alpha increases from alphaTrough";
}

TEST_F(AllocatorFixture, Hysteresis_SerializationRoundTrip) {
    // Stall flags and CL recovery state must survive JSON and proto round-trips.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);
    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);

    // JSON
    {
        const nlohmann::json j = alloc2.serializeJson();
        ASSERT_TRUE(j.contains("stalled"));
        ASSERT_TRUE(j.contains("stalled_neg"));
        ASSERT_TRUE(j.contains("cl_recovering"));
        ASSERT_TRUE(j.contains("cl_recovering_neg"));
        EXPECT_TRUE(j["stalled"].get<bool>());
        EXPECT_NEAR(j["cl_recovering"].get<float>(), lift2.clSep(), 1e-5f);

        LiftCurveModel lift3(gaLiftParams());
        LoadFactorAllocator restored(lift3, kS, kCYb);
        restored.deserializeJson(j);

        LoadFactorInputs in{0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
        const auto out2 = alloc2.solve(in);
        const auto outR = restored.solve(in);
        EXPECT_NEAR(outR.alpha_rad, out2.alpha_rad, 1e-5f);
        EXPECT_EQ(outR.stalled, out2.stalled);
    }

    // Proto (re-inject since alloc2 was consumed above)
    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);
    {
        const std::vector<uint8_t> bytes = alloc2.serializeProto();

        LiftCurveModel lift3(gaLiftParams());
        LoadFactorAllocator restored(lift3, kS, kCYb);
        restored.deserializeProto(bytes);

        LoadFactorInputs in{0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
        const auto out2 = alloc2.solve(in);
        const auto outR = restored.solve(in);
        EXPECT_NEAR(outR.alpha_rad, out2.alpha_rad, 1e-4f);
        EXPECT_EQ(outR.stalled, out2.stalled);
    }
}

// ── Item 3 — CL recovery rate limiting ───────────────────────────────────────

TEST_F(AllocatorFixture, CLRecovery_FlatThroughEntireDescentToAlphaStar) {
    // Standard config: clSep < cl_alpha * alphaStar.
    // While stalled, cl_eff must equal clSep throughout the descent to alphaStar.
    ASSERT_LT(gaLiftParams().cl_sep, gaLiftParams().cl_alpha * lift.alphaStar())
        << "test requires clSep < cl_alpha * alphaStar (typical config)";

    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    // Start stalled at alphaPeak
    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);

    // Step downward until alphaStar is crossed (use kLargeThrust so explicit stall solve advances alpha)
    float alpha_prev = lift2.alphaPeak();
    while (alpha_prev > lift2.alphaStar()) {
        LoadFactorInputs in{0.3f, 0.f, kQ, kLargeThrust, kMass, 0.f, 0.f, kDt};
        const auto out = alloc2.solve(in);
        if (out.stalled) {
            EXPECT_NEAR(out.cl_eff, lift2.clSep(), 1e-5f)
                << "cl_eff must equal clSep throughout descent while stalled";
        }
        alpha_prev = out.alpha_rad;
    }
}

TEST_F(AllocatorFixture, CLRecovery_SnapToNominalBeforeAlphaStar_HighPlateauConfig) {
    // High-plateau config (same as hysteresis snap-down test).
    // cl_eff must snap to the nominal CL at the intersection above alphaStar.
    LiftCurveParams hp;
    hp.cl_alpha              = 5.73f;
    hp.cl_max                = 1.50f;
    hp.cl_min                = -1.50f;
    hp.delta_alpha_stall     = 0.02f;
    hp.delta_alpha_stall_neg = 0.02f;
    hp.cl_sep                = 1.45f;
    hp.cl_sep_neg            = -1.45f;
    LiftCurveModel liftHP(hp);

    ASSERT_GT(liftHP.clSep(), liftHP.clAlpha() * liftHP.alphaStar())
        << "test setup: must be high-plateau config";

    LoadFactorAllocator allocHP(liftHP, kS, kCYb, -kHalfPi, kHalfPi, kAlphaDotMax);
    injectStalledState(allocHP, liftHP.alphaPeak(), liftHP.clSep(), true);

    // Step down to just above alphaStar (snap-down zone)
    const float alpha_snap = liftHP.alphaStar() + 0.001f;
    injectStalledState(allocHP, alpha_snap, liftHP.clSep(), true);

    LoadFactorInputs in{0.3f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
    const auto out = allocHP.solve(in);

    // Stall flag clears via snap-down; cl_eff should equal the nominal CL (not clSep)
    EXPECT_FALSE(out.stalled) << "stall flag should have cleared via snap-down";
    EXPECT_NEAR(out.cl_eff, liftHP.evaluate(out.alpha_rad), 5e-3f)
        << "cl_eff must track nominal CL at the snap-down intersection";
}

TEST_F(AllocatorFixture, CLRecovery_RateLimitedBelowAlphaStar) {
    // After stall clears (alpha < alphaStar), cl_eff must climb toward nominal
    // at exactly cl_alpha * alpha_dot_max * dt per step.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    // Inject stalled state just below alphaStar with cl_recovering = clSep.
    // Use alpha_just_below so that the threshold exit fires on the very first step.
    const float alpha_prev_rad = lift2.alphaStar() - 0.001f;
    injectStalledState(alloc2, alpha_prev_rad, lift2.clSep(), true);

    // Command Nz at alphaStar so alpha stays near alphaStar where cl_nom >> cl_recovering.
    // n_target = cl_alpha * alphaStar * qS / mg keeps Newton at alphaStar after stall clears.
    const float n_target = lift2.clAlpha() * lift2.alphaStar() * kQ * kS / (kMass * kG);
    LoadFactorInputs in{n_target, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};

    // First step: alpha_eq from Newton ≈ alphaStar (stall flag was true but alpha is below);
    // bridge: _stalled=true, max_step=0.005; alpha_out = alpha_prev + clamp(alpha_eq - alpha_prev, -0.005, 0.005).
    // Hysteresis: alpha_out <= alphaStar → flag clears.
    // CL recovery: _stalled now false → _cl_recovering = min(cl_nom, cl_recovering + cl_dot_max*dt).
    const auto out1 = alloc2.solve(in);
    EXPECT_FALSE(out1.stalled) << "stall flag must clear once alpha <= alphaStar";

    // Verify we are in a genuine mid-recovery state: cl_eff < cl_nom at alpha_out.
    const float cl_dot_max    = lift2.clAlpha() * kAlphaDotMax;
    const float expected_step = cl_dot_max * kDt;
    const float cl_before     = out1.cl_eff;
    const float cl_nom_out1   = lift2.evaluate(out1.alpha_rad);
    ASSERT_LT(cl_before, cl_nom_out1) << "test setup: must be in recovery state after first step";

    // Second step at the same Nz: alpha stays near alphaStar, cl_nom stays high.
    // cl_eff must advance by exactly cl_dot_max * dt.
    const auto out2 = alloc2.solve(in);
    const float cl_advance = out2.cl_eff - cl_before;
    EXPECT_NEAR(cl_advance, expected_step, 1e-4f)
        << "cl_eff must advance by cl_dot_max * dt per step during recovery";
}

TEST_F(AllocatorFixture, CLRecovery_InstantDownwardFollow) {
    // During recovery (cl_recovering < cl_nom), if alpha rises back into the
    // descending quadratic, cl_eff must follow cl_nom downward instantly —
    // no rate limit on downward movement.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    // Inject a mid-recovery state: cl_recovering below nominal, not stalled.
    // Alpha near alphaStar (linear region), cl_recovering = clSep (low).
    nlohmann::json j = alloc2.serializeJson();
    j["alpha_prev_rad"] = lift2.alphaStar();
    j["stalled"]        = false;
    j["cl_recovering"]  = lift2.clSep();
    alloc2.deserializeJson(j);

    // Command Nz that pushes alpha up into the descending quadratic (above alpheStar)
    // where cl_nom decreases.  cl_eff should snap down immediately.
    const float alpha_desc    = lift2.alphaPeak() - 0.01f; // in IncipientStallPositive
    const float cl_nom_desc   = lift2.evaluate(alpha_desc);

    // cl_nom_desc > clSep (ascending quadratic), so if cl_recovering = clSep, both
    // clSep and cl_nom_desc are candidates. The formula min(cl_nom, cl_recovering + step)
    // gives min(cl_nom_desc, clSep + step). Since cl_nom_desc > clSep, result = clSep + step.
    // So we verify cl_eff <= cl_nom_desc (doesn't exceed nominal).
    const float n_target = (kQ * kS * lift2.evaluate(alpha_desc)) / (kMass * kG);
    // Approach alpha_desc from below
    alloc2.solve({n_target * 0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt});
    const auto out = alloc2.solve({n_target, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt});

    EXPECT_LE(out.cl_eff, lift2.evaluate(out.alpha_rad) + 1e-4f)
        << "cl_eff must not exceed nominal CL (instant downward snap enforced)";
}

TEST_F(AllocatorFixture, CLRecovery_BridgeDeactivatesAtNominal) {
    // After enough recovery steps, cl_recovering flag must become false and cl_eff
    // must equal the nominal CL.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    // Start recovering from stall at alphaPeak
    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);

    // Run many steps at sub-stall Nz until cl_recovering goes false (use kLargeThrust to advance alpha)
    const int max_steps = 500;
    bool bridge_deactivated = false;
    for (int i = 0; i < max_steps; ++i) {
        LoadFactorInputs in{0.5f, 0.f, kQ, kLargeThrust, kMass, 0.f, 0.f, kDt};
        const auto out = alloc2.solve(in);
        if (!out.cl_recovering) {
            // Verify cl_eff equals nominal
            EXPECT_NEAR(out.cl_eff, lift2.evaluate(out.alpha_rad), 1e-4f)
                << "when bridge deactivates, cl_eff must equal nominal CL";
            bridge_deactivated = true;
            break;
        }
    }
    EXPECT_TRUE(bridge_deactivated) << "CL recovery bridge must deactivate once cl_eff reaches nominal";
}

TEST_F(AllocatorFixture, CLRecovery_NzFromEffectiveCL) {
    // n_z_realized must use cl_eff throughout all phases (stalled, recovering, normal).
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    auto verify_nz_from_cl_eff = [&](const LoadFactorOutputs& out) {
        const float mg  = kMass * kG;
        const float qS  = kQ * kS;
        const float expected = (qS * out.cl_eff + 0.f * std::sin(out.alpha_rad)) / mg;
        EXPECT_NEAR(out.n_z_realized, expected, 1e-5f)
            << "n_z_realized must equal (qS*cl_eff)/mg";
    };

    // Phase 1: stalled
    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);
    verify_nz_from_cl_eff(alloc2.solve({0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt}));

    // Phase 2: recovering (after stall clears)
    for (int i = 0; i < 5; ++i) {
        const auto out = alloc2.solve({0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt});
        verify_nz_from_cl_eff(out);
    }

    // Phase 3: normal pre-stall
    alloc2.reset();
    verify_nz_from_cl_eff(alloc2.solve({1.0f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt}));
}

TEST_F(AllocatorFixture, CLRecovery_TZeroHoldsAlphaWhileStalled) {
    // T=0 while stalled: the explicit solve has no solution (arcsin undefined).
    // alpha_out must be held at _alpha_prev.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    const float alpha_stall = lift2.alphaPeak();
    injectStalledState(alloc2, alpha_stall, lift2.clSep(), true);

    LoadFactorInputs in{0.5f, 0.f, kQ, 0.f /*T=0*/, kMass, 0.f, 0.f, kDt};
    const auto out = alloc2.solve(in);

    // Alpha must not change: with T=0, alpha_eq = alpha_prev (no thrust to drive explicit solve),
    // bridge step = 0, so alpha_out = alpha_stall.
    EXPECT_NEAR(out.alpha_rad, alpha_stall, 1e-5f)
        << "with T=0 and stalled, alpha_eq = alpha_prev so bridge gives zero step";
}

TEST_F(AllocatorFixture, CLRecovery_SerializationRoundTrip) {
    // _cl_recovering and _cl_recovering_neg must survive JSON and proto round-trips.
    LiftCurveModel lift2(gaLiftParams());
    auto alloc2 = makeRateLimited(lift2);

    // Build a mid-recovery state: stall cleared, cl_recovering partially raised.
    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);
    for (int i = 0; i < 3; ++i) {
        alloc2.solve({0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt});
    }

    // JSON round-trip
    {
        const nlohmann::json j = alloc2.serializeJson();
        ASSERT_TRUE(j.contains("cl_recovering"));
        ASSERT_TRUE(j.contains("cl_recovering_neg"));

        LiftCurveModel lift3(gaLiftParams());
        LoadFactorAllocator restored(lift3, kS, kCYb);
        restored.deserializeJson(j);

        LoadFactorInputs in{0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
        EXPECT_NEAR(restored.solve(in).cl_eff, alloc2.solve(in).cl_eff, 1e-5f);
    }

    // Proto round-trip (reinject to get fresh state)
    injectStalledState(alloc2, lift2.alphaPeak(), lift2.clSep(), true);
    for (int i = 0; i < 3; ++i) {
        alloc2.solve({0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt});
    }
    {
        const std::vector<uint8_t> bytes = alloc2.serializeProto();

        LiftCurveModel lift3(gaLiftParams());
        LoadFactorAllocator restored(lift3, kS, kCYb);
        restored.deserializeProto(bytes);

        LoadFactorInputs in{0.5f, 0.f, kQ, 0.f, kMass, 0.f, 0.f, kDt};
        EXPECT_NEAR(restored.solve(in).cl_eff, alloc2.solve(in).cl_eff, 1e-4f);
    }
}
