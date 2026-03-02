#pragma once

#include "aerodynamics/LiftCurveModel.hpp"

// Inputs to the load-factor allocator for a single solve() call.
struct LoadFactorInputs {
    float n;         // commanded normal load factor (g)
    float n_y;       // commanded lateral load factor (g)
    float q_inf;     // dynamic pressure (Pa)
    float thrust_n;  // thrust magnitude (N)
    float mass_kg;   // aircraft mass (kg)
};

// Outputs produced by the load-factor allocator.
struct LoadFactorOutputs {
    float alpha_rad; // angle of attack (rad)
    float beta_rad;  // sideslip angle (rad)
    bool  stall;     // true when α demand has exceeded the pre-stall CL peak
};

// Implicit Newton solver that maps commanded load factors (n, n_y) to angle of
// attack and sideslip.
//
// Normal equation:   f(α) = q·S·C_L(α) + T·sin(α) − n·m·g = 0
// Lateral equation:  g(β) = q·S·C_Yβ·β − T·cos(α)·sin(β) − n_y·m·g = 0
//
// α is solved first; β is solved using the converged α.  Branch continuity is
// maintained by warm-starting each Newton iteration from the previous solution.
class LoadFactorAllocator {
public:
    // liftCurve   — reference must outlive this object
    // S_ref_m2    — reference wing area (m²)
    // cl_y_beta   — lateral force coefficient slope C_Yβ (rad⁻¹, typically < 0)
    explicit LoadFactorAllocator(const LiftCurveModel& liftCurve,
                                 float S_ref_m2,
                                 float cl_y_beta);

    LoadFactorOutputs solve(const LoadFactorInputs& in);

    // Reset warm-start state; call before a discontinuous change in demand.
    void reset(float alpha0_rad = 0.0f, float beta0_rad = 0.0f);

private:
    const LiftCurveModel& _lift;
    float _S;
    float _cl_y_beta;
    float _alpha_prev;
    float _beta_prev;

    static constexpr int   kMaxIter = 20;
    static constexpr float kTol     = 1e-6f;
    static constexpr float kGravity = 9.80665f; // m/s²
};
