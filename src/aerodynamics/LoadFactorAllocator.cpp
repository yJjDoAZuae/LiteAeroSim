#include "aerodynamics/LoadFactorAllocator.hpp"
#include <cmath>

LoadFactorAllocator::LoadFactorAllocator(const LiftCurveModel& liftCurve,
                                         float S_ref_m2,
                                         float cl_y_beta)
    : _lift(liftCurve), _S(S_ref_m2), _cl_y_beta(cl_y_beta),
      _alpha_prev(0.0f), _beta_prev(0.0f) {}

void LoadFactorAllocator::reset(float alpha0_rad, float beta0_rad) {
    _alpha_prev = alpha0_rad;
    _beta_prev  = beta0_rad;
}

LoadFactorOutputs LoadFactorAllocator::solve(const LoadFactorInputs& in) {
    const float mg = in.mass_kg * kGravity;
    const float qS = in.q_inf * _S;
    const float T  = in.thrust_n;

    // ── α solver ─────────────────────────────────────────────────────────────
    // f(α) = q·S·C_L(α) + T·sin(α) − n·m·g = 0
    // f'(α) = q·S·C_L'(α) + T·cos(α)
    float alpha = _alpha_prev;
    bool  positive_stall = false;
    bool  negative_stall = false;

    const float alpha_peak   = _lift.alphaPeak();
    const float alpha_trough = _lift.alphaTrough();
    for (int i = 0; i < kMaxIter; ++i) {
        const float fval   = qS * _lift.evaluate(alpha)   + T * std::sin(alpha) - in.n * mg;
        const float fprime = qS * _lift.derivative(alpha) + T * std::cos(alpha);

        // Fold guard: derivative vanishes at a stall vertex (C_L'=0, T≈0).
        // Determine which vertex by checking the sign of the current α.
        if (std::abs(fprime) < kTol) {
            if (alpha >= 0.0f) {
                positive_stall = true;
                alpha = alpha_peak;
            } else {
                negative_stall = true;
                alpha = alpha_trough;
            }
            break;
        }

        const float alpha_new = alpha - fval / fprime;

        // Overshoot guards: if Newton would cross a stall vertex, demand
        // exceeds the CL envelope and no pre-stall solution exists.
        if (alpha_new > alpha_peak) {
            positive_stall = true;
            alpha = alpha_peak;
            break;
        }
        if (alpha_new < alpha_trough) {
            negative_stall = true;
            alpha = alpha_trough;
            break;
        }

        const bool converged = std::abs(alpha_new - alpha) < kTol;
        alpha = alpha_new;
        if (converged) break;
    }

    _alpha_prev = alpha;

    // ── alphaDot: implicit function theorem on f(α, n) = 0 ───────────────────
    // df/dα · dα/dt + df/dn · dn/dt = 0
    // df/dn = -m·g  →  dα/dt = m·g · n_dot / f'(α)
    const float fprime_alpha = qS * _lift.derivative(alpha) + T * std::cos(alpha);
    float alphaDot = 0.f;
    if (!positive_stall && !negative_stall && std::abs(fprime_alpha) > kTol) {
        alphaDot = mg * in.n_dot / fprime_alpha;
    }

    // ── β solver ─────────────────────────────────────────────────────────────
    // g(β) = q·S·C_Yβ·β − T·cos(α)·sin(β) − n_y·m·g = 0
    // g'(β) = q·S·C_Yβ − T·cos(α)·cos(β)   (always ≤ 0 → unique root)
    float beta      = _beta_prev;
    const float Tca = T * std::cos(alpha);

    for (int i = 0; i < kMaxIter; ++i) {
        const float gval   = qS * _cl_y_beta * beta - Tca * std::sin(beta) - in.n_y * mg;
        const float gprime = qS * _cl_y_beta - Tca * std::cos(beta);

        if (std::abs(gprime) < kTol) break; // degenerate (should not occur for typical params)

        const float beta_new  = beta - gval / gprime;
        const bool  converged = std::abs(beta_new - beta) < kTol;
        beta = beta_new;
        if (converged) break;
    }

    _beta_prev = beta;

    // ── betaDot: implicit function theorem on g(β, n_y, α) = 0 ───────────────
    // dg/dβ · dβ/dt + dg/dα · dα/dt + dg/dn_y · dn_y/dt = 0
    // dg/dβ  = q·S·C_Yβ − T·cos(α)·cos(β)
    // dg/dα  = T·sin(α)·sin(β)
    // dg/dn_y = -m·g
    const float gprime_beta = qS * _cl_y_beta - Tca * std::cos(beta);
    const float dg_dalpha   = T * std::sin(alpha) * std::sin(beta);
    float betaDot = 0.f;
    if (std::abs(gprime_beta) > kTol) {
        betaDot = (mg * in.n_y_dot - dg_dalpha * alphaDot) / gprime_beta;
    }

    return {alpha, beta, _lift.classify(alpha), alphaDot, betaDot};
}
