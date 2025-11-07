#define _USE_MATH_DEFINES

#include "Aerodynamics/AeroPerformance.hpp"
#include <cmath>
#include "Math/math_util.hpp"

const float AeroPerformance::rhoSL = 1.225f; // sea level standard density in kg/m^3
const float AeroPerformance::a_Mach = 340.29f; // speed of sound at sea level in m/s
const float AeroPerformance::g = 9.81f; // standard gravity in m/s^2


float AeroPerformance::AR() const {
    return (bref * bref) / Sref;
}

float AeroPerformance::K() const {
    return 1.0f / (M_PI * AR() * E);
}

float AeroPerformance::CDi(float CL) const {
    return K() * CL * CL;
}

float AeroPerformance::q(float V, float rho) const {
    return 0.5f * rho * V * V;
}

float AeroPerformance::CL(float V, float rho, float N, float mass_kg, AirframePerformance &airframe) const
{
    float L = MathUtil::clip(N, airframe.GMin, airframe.GMax) * mass_kg * g;
    return MathUtil::clip(L / (q(V, rho) * Sref), CLmin, CLmax);
}

// CD
float AeroPerformance::CD(float CL) const
{
    return CD0 + CDi(CL);
}

// this is a total drag
float AeroPerformance::Drag(float V, float rho, float N, float mass_kg, AirframePerformance &airframe) const
{
    float i_CD = CD(CL(V,rho,N,mass_kg,airframe));

    return i_CD * q(V,rho) * Sref;
}

float AeroPerformance::Lift(float V, float rho, float N, float mass_kg, AirframePerformance &airframe) const
{
    return CL(V,rho,N,mass_kg,airframe) * q(V,rho) * Sref;
}

// TODO: check this
float AeroPerformance::CLalpha(float rho, float V) const {
    return (2.0f * M_PI * AR()) / (2.0f + sqrt(4.0f + pow((AR() * beta(rho, V) / E), 2.0f))) * (Sref / q(V,rho));
}

float AeroPerformance::beta(float rho, float V) const {
    return sqrt(1 - Mach(V) * Mach(V));
}

float AeroPerformance::EAS(float TAS, float rho) {
    return TAS * sqrt(rhoSL / rhoSL);
}

float AeroPerformance::EAS2TAS(float EAS, float rho) {
    return EAS / sqrt(rhoSL / rhoSL);
}

float AeroPerformance::Mach(float TAS) {
    return TAS / a_Mach;
}
