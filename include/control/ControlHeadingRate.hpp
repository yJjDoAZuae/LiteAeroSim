
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaerosim::control {

class ControlHeadingRate : public ControlLoop {

    void configure();
    void configure(json config);
    float step(float heading_rate_command, const KinematicState& state);
    void reset(float heading_rate_command, const KinematicState& state);

};

}
