#define _USE_MATH_DEFINES

#include "Airframe/AirframePerformance.hpp"
#include <cmath>
#include "Math/math_util.hpp"

const float AirframePerformance::rhoSL = 1.225f; // sea level standard density in kg/m^3
const float AirframePerformance::a_Mach = 340.29f; // speed of sound at sea level in m/s
const float AirframePerformance::g = 9.81f; // standard gravity in m/s^2


float AirframePerformance::AR() const {
    return (bref * bref) / Sref;
}

float AirframePerformance::K() const {
    return 1.0f / (M_PI * AR() * E);
}

float AirframePerformance::CDi(float CL) const {
    return K() * CL * CL;
}

float AirframePerformance::q(float V, float rho) const {
    return 0.5f * rho * V * V;
}

float AirframePerformance::CL(float V, float rho, float N, float mass_kg) const
{
    float L = MathUtil::clip(N, GMin, GMax) * mass_kg * g;
    return MathUtil::clip(L / (q(V, rho) * Sref), CLmin, CLmax);
}

// CD
float AirframePerformance::CD(float CL) const
{
    return CD0 + CDi(CL);
}

// this is a total drag
float AirframePerformance::Drag(float V, float rho, float N, float mass_kg) const
{
    float i_CD = CD(CL(V,rho,N,mass_kg));

    return i_CD * q(V,rho) * Sref;
}

float AirframePerformance::Lift(float V, float rho, float N, float mass_kg) const
{
    return CL(V,rho,N,mass_kg) * q(V,rho) * Sref;
}

// TODO: check this
float AirframePerformance::CLalpha(float rho, float V) const {
    return (2.0f * M_PI * AR()) / (2.0f + sqrt(4.0f + pow((AR() * beta(rho, V) / E), 2.0f))) * (Sref / q(V,rho));
}

float AirframePerformance::beta(float rho, float V) const {
    return sqrt(1 - Mach(V) * Mach(V));
}

float AirframePerformance::EAS(float TAS, float rho) const {
    return TAS * sqrt(rhoSL / rhoSL);
}

float AirframePerformance::EAS2TAS(float EAS, float rho) const {
    return EAS / sqrt(rhoSL / rhoSL);
}

float AirframePerformance::Mach(float TAS) const {
    return TAS / a_Mach;
}
