#include "aerodynamics/LiftCurveModel.hpp"

LiftCurveModel::LiftCurveModel(const LiftCurveParams& params) : _p(params) {
    computeCoefficients();
}

void LiftCurveModel::computeCoefficients() {
    // ── Positive stall ────────────────────────────────────────────────────────
    // Downward-opening parabola with vertex at (_alpha_peak, cl_max).
    // C¹ joined to the linear model at _alpha_star.
    _alpha_peak = _p.cl_max / _p.cl_alpha + _p.delta_alpha_stall / 2.0f;
    _alpha_star  = _alpha_peak - _p.delta_alpha_stall;

    _a2 = -_p.cl_alpha / (2.0f * _p.delta_alpha_stall);
    _a1 = -2.0f * _a2 * _alpha_peak;
    _a0 = _a2 * _alpha_peak * _alpha_peak + _p.cl_max;

    // α_sep: descending side where C_L = cl_sep
    _alpha_sep = _alpha_peak + std::sqrt((_p.cl_sep - _p.cl_max) / _a2);

    // ── Negative stall ────────────────────────────────────────────────────────
    // Upward-opening parabola with vertex at (_alpha_peak_neg, cl_min).
    // C¹ joined to the linear model at _alpha_star_neg.
    _alpha_peak_neg = _p.cl_min / _p.cl_alpha - _p.delta_alpha_stall_neg / 2.0f;
    _alpha_star_neg  = _alpha_peak_neg + _p.delta_alpha_stall_neg;

    _a2n = _p.cl_alpha / (2.0f * _p.delta_alpha_stall_neg);
    _a1n = -2.0f * _a2n * _alpha_peak_neg;
    _a0n = _a2n * _alpha_peak_neg * _alpha_peak_neg + _p.cl_min;

    // α_sep_neg: ascending side (going more negative) where C_L = cl_sep_neg
    _alpha_sep_neg = _alpha_peak_neg - std::sqrt((_p.cl_sep_neg - _p.cl_min) / _a2n);
}

float LiftCurveModel::evaluate(float alpha_rad) const {
    if (alpha_rad < _alpha_sep_neg) {
        return _p.cl_sep_neg;
    }
    if (alpha_rad < _alpha_star_neg) {
        return _a2n * alpha_rad * alpha_rad + _a1n * alpha_rad + _a0n;
    }
    if (alpha_rad <= _alpha_star) {
        return _p.cl_alpha * alpha_rad;
    }
    if (alpha_rad <= _alpha_sep) {
        return _a2 * alpha_rad * alpha_rad + _a1 * alpha_rad + _a0;
    }
    return _p.cl_sep;
}

float LiftCurveModel::derivative(float alpha_rad) const {
    if (alpha_rad < _alpha_sep_neg) {
        return 0.0f;
    }
    if (alpha_rad < _alpha_star_neg) {
        return 2.0f * _a2n * alpha_rad + _a1n;
    }
    if (alpha_rad <= _alpha_star) {
        return _p.cl_alpha;
    }
    if (alpha_rad <= _alpha_sep) {
        return 2.0f * _a2 * alpha_rad + _a1;
    }
    return 0.0f;
}

float LiftCurveModel::alphaPeak() const {
    return _alpha_peak;
}

float LiftCurveModel::alphaTrough() const {
    return _alpha_peak_neg;
}
