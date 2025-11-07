// structural performance limitation model of the aircraft

#pragma once

class AirframePerformance {

    public:

        float GMax;
        float GMin;
        float TASMax;
        float MachMax;
        float CLmax;
        float CLmin;

        float Sref; // reference area in m^2
        float bref; // wingspan in m
        float cref; // mean aerodynamic chord in m
        
        float E; // Oswald efficiency factor
        float CD0; // zero-lift drag coefficient

        float AR() const;
        float K() const;
        float CDi(float CL) const;
        float CL(float V, float rho, float N, float mass_kg) const;
        float CLalpha(float rho, float V) const;
        float beta(float rho, float V) const;
        float q(float V, float rho) const;
        float CD(float CL) const;

        float Drag(float V, float rho, float N, float mass_kg) const;
        float Lift(float V, float rho, float N, float mass_kg) const;

        float EAS(float TAS, float rho) const;
        float EAS2TAS(float EAS, float rho) const;
        float Mach(float TAS) const;

        static const float rhoSL;
        static const float a_Mach; // speed of sound at sea level in m/s
        static const float g; // standard gravity in m/s^2

};
