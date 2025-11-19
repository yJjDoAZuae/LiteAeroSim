// structural performance limitation model of the aircraft

#pragma once

class AirframePerformance
{

    public:

        AirframePerformance() : GMax(0), GMin(0), TASMax(0), MachMax(0) {}

        float GMax;
        float GMin;
        float TASMax;
        float MachMax;

};
