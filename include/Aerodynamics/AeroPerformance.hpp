#pragma once

#include "Airframe/AirframePerformance.hpp"

// Aerodynamic performance model for the aircraft
// determines load factor and roll rate capabilities at each iteration

class AeroPerformance {

    public:

        float CLmax; // maximum lift coefficient in the wind Z axis
        float CLmin; // minimum lift coefficient in the wind Z axis
        float Cmxmax; // maximum roll moment coefficient in the wind X axis
        float Cmymax; // maximum pitch moment coefficient in the wind Y axis

        float Sref; // reference area in m^2
        float bref; // wingspan in m
        float cref; // mean aerodynamic chord in m
        
        float E; // Oswald efficiency factor
        float CD0; // zero-lift drag coefficient

        float AR() const;
        float K() const;
        float CDi(float CL) const;
        float CL(float V, float rho, float N, float mass_kg, AirframePerformance &airframe) const;
        float CLalpha(float rho, float V) const;
        float beta(float rho, float V) const; // prandtl-glauert factor
        float q(float V, float rho) const;
        float CD(float CL) const;

        float Drag(float V, float rho, float N, float mass_kg, AirframePerformance &airframe) const;
        float Lift(float V, float rho, float N, float mass_kg, AirframePerformance &airframe) const;

        static float EAS(float TAS, float rho);
        static float EAS2TAS(float EAS, float rho);
        static float Mach(float TAS);

        static const float rhoSL;
        static const float a_Mach; // speed of sound at sea level in m/s
        static const float g; // standard gravity in m/s^2

};
