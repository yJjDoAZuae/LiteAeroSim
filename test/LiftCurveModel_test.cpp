#define _USE_MATH_DEFINES
#include "aerodynamics/LiftCurveModel.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>

// Symmetric general-aviation lift curve parameters
//   cl_alpha          = 5.73   rad^-1
//   cl_max            = 1.20              positive peak
//   cl_min            = -1.20             negative peak
//   delta_alpha_stall = 0.05   rad        angular half-width of stall break (both sides)
//   cl_sep            = 0.80              positive plateau
//   cl_sep_neg        = -0.80             negative plateau
//
// Derived (positive side):
//   alpha_peak    = cl_max/cl_alpha + delta_alpha_stall/2  ≈  0.23442 rad
//   alpha_star    = alpha_peak - delta_alpha_stall          ≈  0.18442 rad
//   alpha_sep     ≈  0.31797 rad
//
// Derived (negative side, symmetric):
//   alpha_peak_neg ≈ -0.23442 rad
//   alpha_star_neg ≈ -0.18442 rad
//   alpha_sep_neg  ≈ -0.31797 rad

static LiftCurveParams gaParams() {
    return {5.73f, 1.20f, -1.20f, 0.05f, 0.05f, 0.80f, -0.80f};
}

static float alphaStarFromParams(const LiftCurveParams& p) {
    return p.cl_max / p.cl_alpha - p.delta_alpha_stall / 2.0f;
}

static float alphaSepFromParams(const LiftCurveParams& p) {
    const float alpha_peak = p.cl_max / p.cl_alpha + p.delta_alpha_stall / 2.0f;
    const float a2 = -p.cl_alpha / (2.0f * p.delta_alpha_stall);
    return alpha_peak + std::sqrt((p.cl_sep - p.cl_max) / a2);
}

static float alphaStarNegFromParams(const LiftCurveParams& p) {
    return p.cl_min / p.cl_alpha + p.delta_alpha_stall_neg / 2.0f;
}

static float alphaSepNegFromParams(const LiftCurveParams& p) {
    const float alpha_peak_neg = p.cl_min / p.cl_alpha - p.delta_alpha_stall_neg / 2.0f;
    const float a2_neg = p.cl_alpha / (2.0f * p.delta_alpha_stall_neg);
    return alpha_peak_neg - std::sqrt((p.cl_sep_neg - p.cl_min) / a2_neg);
}

// ── Positive-stall side ──────────────────────────────────────────────────────

TEST(LiftCurveModelTest, ZeroAlpha) {
    LiftCurveModel m(gaParams());
    EXPECT_NEAR(m.evaluate(0.0f),   0.0f, 1e-5f);
    EXPECT_NEAR(m.derivative(0.0f), 5.73f, 1e-5f);
}

TEST(LiftCurveModelTest, LinearRegionPositive) {
    LiftCurveModel m(gaParams());
    // α = 0.10 rad — below alpha_star ≈ 0.184 and above alpha_star_neg ≈ -0.184
    EXPECT_NEAR(m.evaluate(0.10f),   5.73f * 0.10f, 1e-5f);
    EXPECT_NEAR(m.derivative(0.10f), 5.73f,          1e-5f);
}

TEST(LiftCurveModelTest, C1ContinuityAtAlphaStar) {
    LiftCurveModel m(gaParams());
    const float eps = 1e-5f;
    const float alpha_star = alphaStarFromParams(gaParams());
    EXPECT_NEAR(m.evaluate(alpha_star - eps),   m.evaluate(alpha_star + eps),   2e-4f);
    EXPECT_NEAR(m.derivative(alpha_star - eps), m.derivative(alpha_star + eps), 2e-3f);
    EXPECT_NEAR(m.derivative(alpha_star), 5.73f, 1e-4f);
}

TEST(LiftCurveModelTest, PeakCLAndZeroSlope) {
    LiftCurveModel m(gaParams());
    const float alpha_peak = gaParams().cl_max / gaParams().cl_alpha + gaParams().delta_alpha_stall / 2.0f;
    EXPECT_NEAR(m.evaluate(alpha_peak),   gaParams().cl_max, 1e-4f);
    EXPECT_NEAR(m.derivative(alpha_peak), 0.0f,              1e-4f);
}

TEST(LiftCurveModelTest, QuadraticRegionPositive) {
    LiftCurveModel m(gaParams());
    // Test at an interior point of Region 4 (positive quadratic, not the vertex).
    const LiftCurveParams p = gaParams();
    const float alpha_star = alphaStarFromParams(p);
    const float alpha_sep  = alphaSepFromParams(p);
    const float alpha_test = 0.5f * (alpha_star + alpha_sep);

    const float alpha_peak = p.cl_max / p.cl_alpha + p.delta_alpha_stall / 2.0f;
    const float a2 = -p.cl_alpha / (2.0f * p.delta_alpha_stall);
    const float a1 = -2.0f * a2 * alpha_peak;
    const float a0 = a2 * alpha_peak * alpha_peak + p.cl_max;

    EXPECT_NEAR(m.evaluate(alpha_test),   a2*alpha_test*alpha_test + a1*alpha_test + a0, 1e-5f);
    EXPECT_NEAR(m.derivative(alpha_test), 2.0f*a2*alpha_test + a1,                       1e-5f);
}

TEST(LiftCurveModelTest, C0JoinAtAlphaSep) {
    LiftCurveModel m(gaParams());
    const float eps = 1e-5f;
    const float alpha_sep = alphaSepFromParams(gaParams());
    EXPECT_NEAR(m.evaluate(alpha_sep - eps), m.evaluate(alpha_sep + eps), 3e-4f);
    EXPECT_NEAR(m.evaluate(alpha_sep + eps), gaParams().cl_sep, 1e-4f);
    const float slope_quad = m.derivative(alpha_sep - eps);
    const float slope_flat = m.derivative(alpha_sep + eps);
    EXPECT_LT(slope_quad, -1.0f);
    EXPECT_NEAR(slope_flat, 0.0f, 1e-5f);
    EXPECT_GT(std::abs(slope_quad - slope_flat), 1.0f);
}

TEST(LiftCurveModelTest, FlatRegionPositive) {
    LiftCurveModel m(gaParams());
    EXPECT_NEAR(m.evaluate(0.40f),   gaParams().cl_sep, 1e-5f);
    EXPECT_NEAR(m.evaluate(0.60f),   gaParams().cl_sep, 1e-5f);
    EXPECT_NEAR(m.derivative(0.40f), 0.0f, 1e-5f);
}

TEST(LiftCurveModelTest, AlphaPeakReturnsExpectedAngle) {
    LiftCurveModel m(gaParams());
    const float expected = gaParams().cl_max / gaParams().cl_alpha + gaParams().delta_alpha_stall / 2.0f;
    EXPECT_NEAR(m.alphaPeak(), expected, 1e-5f);
}

// ── Negative-stall side ──────────────────────────────────────────────────────

TEST(LiftCurveModelTest, LinearRegionNegative) {
    LiftCurveModel m(gaParams());
    // α = -0.10 rad — above alpha_star_neg ≈ -0.184, still linear
    EXPECT_NEAR(m.evaluate(-0.10f),   5.73f * -0.10f, 1e-5f);
    EXPECT_NEAR(m.derivative(-0.10f), 5.73f,           1e-5f);
}

TEST(LiftCurveModelTest, C1ContinuityAtAlphaStarNeg) {
    LiftCurveModel m(gaParams());
    const float eps = 1e-5f;
    const float alpha_star_neg = alphaStarNegFromParams(gaParams());
    // Approaching from the less-negative (linear) side vs. the more-negative (quadratic) side
    EXPECT_NEAR(m.evaluate(alpha_star_neg + eps),   m.evaluate(alpha_star_neg - eps),   2e-4f);
    EXPECT_NEAR(m.derivative(alpha_star_neg + eps), m.derivative(alpha_star_neg - eps), 2e-3f);
    EXPECT_NEAR(m.derivative(alpha_star_neg), 5.73f, 1e-4f);
}

TEST(LiftCurveModelTest, TroughCLAndZeroSlope) {
    LiftCurveModel m(gaParams());
    const float alpha_trough = gaParams().cl_min / gaParams().cl_alpha - gaParams().delta_alpha_stall_neg / 2.0f;
    EXPECT_NEAR(m.evaluate(alpha_trough),   gaParams().cl_min, 1e-4f);
    EXPECT_NEAR(m.derivative(alpha_trough), 0.0f,              1e-4f);
}

TEST(LiftCurveModelTest, QuadraticRegionNegative) {
    LiftCurveModel m(gaParams());
    // Test at an interior point of Region 2 (negative quadratic, not the vertex).
    const LiftCurveParams p = gaParams();
    const float alpha_star_neg = alphaStarNegFromParams(p);
    const float alpha_sep_neg  = alphaSepNegFromParams(p);
    const float alpha_test = 0.5f * (alpha_star_neg + alpha_sep_neg);

    const float alpha_peak_neg = p.cl_min / p.cl_alpha - p.delta_alpha_stall_neg / 2.0f;
    const float a2n = p.cl_alpha / (2.0f * p.delta_alpha_stall_neg);
    const float a1n = -2.0f * a2n * alpha_peak_neg;
    const float a0n = a2n * alpha_peak_neg * alpha_peak_neg + p.cl_min;

    EXPECT_NEAR(m.evaluate(alpha_test),   a2n*alpha_test*alpha_test + a1n*alpha_test + a0n, 1e-5f);
    EXPECT_NEAR(m.derivative(alpha_test), 2.0f*a2n*alpha_test + a1n,                        1e-5f);
}

TEST(LiftCurveModelTest, C0JoinAtAlphaSepNeg) {
    LiftCurveModel m(gaParams());
    const float eps = 1e-5f;
    const float alpha_sep_neg = alphaSepNegFromParams(gaParams());
    // Value continuous across the boundary
    EXPECT_NEAR(m.evaluate(alpha_sep_neg + eps), m.evaluate(alpha_sep_neg - eps), 3e-4f);
    EXPECT_NEAR(m.evaluate(alpha_sep_neg - eps), gaParams().cl_sep_neg, 1e-4f);
    // Slope is NOT continuous
    const float slope_quad = m.derivative(alpha_sep_neg + eps);
    const float slope_flat = m.derivative(alpha_sep_neg - eps);
    // At the left boundary of the quadratic (alpha < alpha_peak_neg), the upward-opening
    // parabola has a negative slope — symmetrically steep to the positive-side join.
    EXPECT_LT(slope_quad, -1.0f);
    EXPECT_NEAR(slope_flat, 0.0f, 1e-5f);
    EXPECT_GT(std::abs(slope_quad - slope_flat), 1.0f);
}

TEST(LiftCurveModelTest, FlatRegionNegative) {
    LiftCurveModel m(gaParams());
    EXPECT_NEAR(m.evaluate(-0.40f),   gaParams().cl_sep_neg, 1e-5f);
    EXPECT_NEAR(m.evaluate(-0.60f),   gaParams().cl_sep_neg, 1e-5f);
    EXPECT_NEAR(m.derivative(-0.40f), 0.0f, 1e-5f);
}

TEST(LiftCurveModelTest, AlphaTroughReturnsExpectedAngle) {
    LiftCurveModel m(gaParams());
    const float expected = gaParams().cl_min / gaParams().cl_alpha - gaParams().delta_alpha_stall_neg / 2.0f;
    EXPECT_NEAR(m.alphaTrough(), expected, 1e-5f);
}

// ── classify() ───────────────────────────────────────────────────────────────

TEST(LiftCurveModelTest, ClassifyLinear) {
    LiftCurveModel m(gaParams());
    EXPECT_EQ(m.classify(0.0f),   LiftCurveSegment::Linear);
    EXPECT_EQ(m.classify(0.10f),  LiftCurveSegment::Linear);
    EXPECT_EQ(m.classify(-0.10f), LiftCurveSegment::Linear);
}

TEST(LiftCurveModelTest, ClassifyIncipientStallPositive) {
    LiftCurveModel m(gaParams());
    const float alpha_star = alphaStarFromParams(gaParams());
    const float alpha_peak = gaParams().cl_max / gaParams().cl_alpha + gaParams().delta_alpha_stall / 2.0f;
    const float alpha_mid  = 0.5f * (alpha_star + alpha_peak);
    EXPECT_EQ(m.classify(alpha_mid),    LiftCurveSegment::IncipientStallPositive);
    EXPECT_EQ(m.classify(m.alphaPeak()), LiftCurveSegment::IncipientStallPositive);
}

TEST(LiftCurveModelTest, ClassifyPostStallPositive) {
    LiftCurveModel m(gaParams());
    const float alpha_mid = 0.5f * (m.alphaPeak() + alphaSepFromParams(gaParams()));
    EXPECT_EQ(m.classify(alpha_mid), LiftCurveSegment::PostStallPositive);
}

TEST(LiftCurveModelTest, ClassifyFullySeparatedPositive) {
    LiftCurveModel m(gaParams());
    EXPECT_EQ(m.classify(0.40f), LiftCurveSegment::FullySeparatedPositive);
    EXPECT_EQ(m.classify(1.00f), LiftCurveSegment::FullySeparatedPositive);
}

TEST(LiftCurveModelTest, ClassifyIncipientStallNegative) {
    LiftCurveModel m(gaParams());
    const float alpha_mid = 0.5f * (m.alphaTrough() + alphaStarNegFromParams(gaParams()));
    EXPECT_EQ(m.classify(alpha_mid),     LiftCurveSegment::IncipientStallNegative);
    EXPECT_EQ(m.classify(m.alphaTrough()), LiftCurveSegment::IncipientStallNegative);
}

TEST(LiftCurveModelTest, ClassifyPostStallNegative) {
    LiftCurveModel m(gaParams());
    const float alpha_mid = 0.5f * (alphaSepNegFromParams(gaParams()) + m.alphaTrough());
    EXPECT_EQ(m.classify(alpha_mid), LiftCurveSegment::PostStallNegative);
}

TEST(LiftCurveModelTest, ClassifyFullySeparatedNegative) {
    LiftCurveModel m(gaParams());
    EXPECT_EQ(m.classify(-0.40f), LiftCurveSegment::FullySeparatedNegative);
    EXPECT_EQ(m.classify(-1.00f), LiftCurveSegment::FullySeparatedNegative);
}

// ── Degenerate and invalid construction ──────────────────────────────────────

TEST(LiftCurveModelTest, DegeneratePositiveClSepEqualsClMax) {
    // cl_sep == cl_max: post-stall descending region has zero width; the flat
    // plateau begins immediately at the peak. Construction must succeed.
    LiftCurveParams p = gaParams();
    p.cl_sep = p.cl_max;
    LiftCurveModel m(p);

    const float alpha_peak = m.alphaPeak();
    // Value at the peak equals cl_max; flat plateau continues for α > α_peak.
    EXPECT_NEAR(m.evaluate(alpha_peak),         p.cl_max, 1e-5f);
    EXPECT_NEAR(m.evaluate(alpha_peak + 0.01f), p.cl_max, 1e-5f);
    // At the peak: IncipientStallPositive (α ≤ α_peak).
    // Just above: FullySeparatedPositive (α_sep == α_peak, so PostStallPositive unreachable).
    EXPECT_EQ(m.classify(alpha_peak),          LiftCurveSegment::IncipientStallPositive);
    EXPECT_EQ(m.classify(alpha_peak + 0.001f), LiftCurveSegment::FullySeparatedPositive);
}

TEST(LiftCurveModelTest, InvalidPositiveClSepAboveClMax) {
    // cl_sep > cl_max: separation point would require sqrt of a negative — must throw.
    LiftCurveParams p = gaParams();
    p.cl_sep = p.cl_max + 0.1f;
    EXPECT_THROW(LiftCurveModel{p}, std::invalid_argument);
}

TEST(LiftCurveModelTest, DegenerateNegativeClSepNegEqualsClMin) {
    // cl_sep_neg == cl_min: symmetric to the positive degenerate case.
    LiftCurveParams p = gaParams();
    p.cl_sep_neg = p.cl_min;
    LiftCurveModel m(p);

    const float alpha_trough = m.alphaTrough();
    EXPECT_NEAR(m.evaluate(alpha_trough),           p.cl_min, 1e-5f);
    EXPECT_NEAR(m.evaluate(alpha_trough - 0.01f),   p.cl_min, 1e-5f);
    EXPECT_EQ(m.classify(alpha_trough),           LiftCurveSegment::IncipientStallNegative);
    EXPECT_EQ(m.classify(alpha_trough - 0.001f),  LiftCurveSegment::FullySeparatedNegative);
}

TEST(LiftCurveModelTest, InvalidNegativeClSepNegBelowClMin) {
    // cl_sep_neg < cl_min: separation point would require sqrt of a negative — must throw.
    LiftCurveParams p = gaParams();
    p.cl_sep_neg = p.cl_min - 0.1f;
    EXPECT_THROW(LiftCurveModel{p}, std::invalid_argument);
}

// ── Symmetry ─────────────────────────────────────────────────────────────────

TEST(LiftCurveModelTest, SymmetryForSymmetricParams) {
    LiftCurveModel m(gaParams());
    // For a perfectly symmetric model: C_L(-α) = -C_L(α) and C_L'(-α) = C_L'(α)
    const float test_angles[] = {0.05f, 0.10f, 0.20f, 0.25f, 0.32f, 0.45f};
    for (float a : test_angles) {
        EXPECT_NEAR(m.evaluate(-a), -m.evaluate(a),     1e-4f) << "at alpha = " << a;
        EXPECT_NEAR(m.derivative(-a), m.derivative(a),  1e-4f) << "at alpha = " << a;
    }
}
