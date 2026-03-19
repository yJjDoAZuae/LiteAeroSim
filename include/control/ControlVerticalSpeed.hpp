
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaerosim::control {

// input is a vertical speed command (positive local level up)
// output is a vertical acceleration command (positive local level up)
class ControlVerticalSpeed : public ControlLoop {

    void configure();
    void configure(json config);
    float step(float command, const KinematicState& state);
    void reset(float command, const KinematicState& state);

};

}
