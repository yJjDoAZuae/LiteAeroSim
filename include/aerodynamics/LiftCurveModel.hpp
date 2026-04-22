#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <nlohmann/json.hpp>

// Identifies which of the seven segments of the piecewise lift curve contains α.
// Ordinal values increase monotonically with α, so comparisons such as
// (segment >= LiftCurveSegment::IncipientStallPositive) are meaningful.
enum class LiftCurveSegment {
    FullySeparatedNegative = -3,  // α < α_sep_neg (flat negative plateau)
    PostStallNegative      = -2,  // α_sep_neg ≤ α < α_trough (descending toward trough)
    IncipientStallNegative = -1,  // α_trough ≤ α < α_*_neg (recovering toward linear)
    Linear                 =  0,  // α_*_neg ≤ α ≤ α_* (C_Lα·α)
    IncipientStallPositive =  1,  // α_* < α ≤ α_peak (approaching peak)
    PostStallPositive      =  2,  // α_peak < α ≤ α_sep (descending from peak)
    FullySeparatedPositive =  3,  // α > α_sep (flat positive plateau)
};

// Parameters defining the five-region piecewise lift curve C_L(α).
// See docs/algorithms/equations_of_motion.md — Lift Curve Model.
struct LiftCurveParams {
    float cl_alpha;              // pre-stall lift-curve slope (rad^-1)
    float cl_max;                // peak lift coefficient (positive stall vertex)
    float cl_min;                // minimum lift coefficient (negative stall vertex, cl_min < 0)
    float delta_alpha_stall;     // angular distance from positive linear join to positive stall vertex (rad)
    float delta_alpha_stall_neg; // angular distance from negative linear join to negative stall vertex (rad)
    float cl_sep;                // positive post-stall plateau (cl_sep < cl_max)
    float cl_sep_neg;            // negative post-stall plateau (cl_sep_neg > cl_min)
};

// Five-region piecewise lift curve model.
//
// Region 1 (α < α_sep_neg)       : C_L = cl_sep_neg            (negative flat plateau)
// Region 2 (α_sep_neg ≤ α < α_*_neg) : C_L = a2n·α²+a1n·α+a0n  (negative quadratic, upward-opening)
// Region 3 (α_*_neg ≤ α ≤ α_*)   : C_L = cl_alpha · α          (linear, C¹ joins at both ends)
// Region 4 (α_* < α ≤ α_sep)     : C_L = a2·α²+a1·α+a0         (positive quadratic, downward-opening)
// Region 5 (α > α_sep)           : C_L = cl_sep                 (positive flat plateau)
//
// α_* and α_sep (and their negative counterparts) are derived from the seven input parameters.
// Requires cl_sep ≤ cl_max, cl_sep_neg ≥ cl_min, delta_alpha_stall > 0, delta_alpha_stall_neg > 0.
// When cl_sep == cl_max (or cl_sep_neg == cl_min) the post-stall descending region has zero width
// and the flat plateau begins immediately at the peak (or trough). The constructor throws
// std::invalid_argument if cl_sep > cl_max or cl_sep_neg < cl_min.
//
// Stateless; construct once per aircraft configuration.
class LiftCurveModel {
public:
    explicit LiftCurveModel(const LiftCurveParams& params);

    float evaluate(float alpha_rad) const;   // C_L(α)
    float derivative(float alpha_rad) const; // dC_L/dα

    // Angle (rad) at which the positive quadratic reaches cl_max.
    float alphaPeak()    const;

    // Angle (rad) at which the linear region meets the positive quadratic (C¹ join).
    float alphaStar()    const;

    // Angle (rad) at which the negative quadratic reaches cl_min.
    float alphaTrough()  const;

    // Angle (rad) at which the linear region meets the negative quadratic (C¹ join).
    float alphaStarNeg() const;

    // Angle (rad) at which the positive descending parabola meets the flat plateau (cl_sep).
    float alphaSep()     const;

    // Angle (rad) at which the negative descending parabola meets the flat plateau (cl_sep_neg).
    float alphaSepNeg()  const;

    float clAlpha()      const;  // pre-stall lift-curve slope C_Lα (rad⁻¹)
    float clSep()        const;  // positive post-stall plateau CL
    float clSepNeg()     const;  // negative post-stall plateau CL

    // Returns which piecewise segment contains alpha_rad.
    LiftCurveSegment classify(float alpha_rad) const;

    nlohmann::json        serializeJson()                               const;
    static LiftCurveModel deserializeJson(const nlohmann::json&         j);

    std::vector<uint8_t>  serializeProto()                             const;
    static LiftCurveModel deserializeProto(const std::vector<uint8_t>&  bytes);

private:
    LiftCurveParams _p;

    // Positive-stall derived values
    float _alpha_star;
    float _alpha_peak;
    float _alpha_sep;
    float _a2, _a1, _a0;

    // Negative-stall derived values
    float _alpha_star_neg;
    float _alpha_peak_neg;
    float _alpha_sep_neg;
    float _a2n, _a1n, _a0n;

    void computeCoefficients();
};
