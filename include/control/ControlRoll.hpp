
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaerosim::control {

// input command is roll angle about velocity vector
// output command is roll rate about velocity vector

class ControlRoll : public ControlLoop {

    void configure();
    void configure(json config);
    float step(float command, const KinematicState& state);
    void reset(float command, const KinematicState& state);

};

}
