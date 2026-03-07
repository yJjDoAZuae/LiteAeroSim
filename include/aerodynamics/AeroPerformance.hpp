#pragma once

#include <cstdint>
#include <vector>
#include <nlohmann/json.hpp>

namespace liteaerosim::aerodynamics {

// Wind-frame aerodynamic forces (SI units: Newtons).
// Sign convention (NED-wind frame, X forward, Y right, Z down):
//   x_n — drag force: negative (opposes motion)
//   y_n — side force: positive right
//   z_n — lift force: negative (lift acts upward, opposing positive Z)
struct AeroForces {
    float x_n = 0.f;
    float y_n = 0.f;
    float z_n = 0.f;
};

// Stateless aerodynamic force model for the simple drag-polar + lateral-force model.
//
// Normal force equation (z-axis, wind frame):
//   Fz = -q · S · CL          (lift upward = negative wind Z)
//
// Drag polar (x-axis, wind frame):
//   CDi = k · CL²             (k = 1 / (π · e · AR))
//   CD  = CD0 + CDi
//   Fx  = -q · S · CD         (drag opposes motion = negative wind X)
//
// Lateral force (y-axis, wind frame):
//   CY  = C_Yβ · β
//   Fy  =  q · S · CY
//
// Construct once per aircraft configuration.
class AeroPerformance {
public:
    // S_ref_m2  — reference wing area (m²)
    // ar        — aspect ratio b²/S
    // e         — Oswald efficiency factor (0 < e ≤ 1)
    // cd0       — zero-lift drag coefficient
    // cl_y_beta — lateral force slope C_Yβ (rad⁻¹, typically < 0)
    AeroPerformance(float S_ref_m2, float ar, float e, float cd0, float cl_y_beta);

    // Compute aerodynamic forces in the Wind frame.
    //   alpha_rad — angle of attack (rad), from LoadFactorAllocator::solve()
    //   beta_rad  — sideslip angle (rad), from LoadFactorAllocator::solve()
    //   q_inf_pa  — dynamic pressure (Pa)
    //   cl        — lift coefficient from LiftCurveModel::evaluate(alpha_rad)
    AeroForces compute(float alpha_rad, float beta_rad, float q_inf_pa, float cl) const;

    // Drag polar diagnostics
    float cd0()           const { return _cd0; }
    float inducedDragK()  const { return _k; }

    // Serialization (stateless — serializes constructor parameters)
    nlohmann::json        serializeJson()                               const;
    static AeroPerformance deserializeJson(const nlohmann::json&        j);

    std::vector<uint8_t>  serializeProto()                             const;
    static AeroPerformance deserializeProto(const std::vector<uint8_t>& bytes);

private:
    float _S;           // reference wing area (m²)
    float _ar;          // aspect ratio
    float _e;           // Oswald efficiency
    float _cd0;         // zero-lift drag coefficient
    float _k;           // induced drag constant = 1 / (π · e · AR)
    float _cl_y_beta;   // lateral force slope (rad⁻¹)
};

} // namespace liteaerosim::aerodynamics
