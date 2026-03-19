
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaerosim::control {

class ControlLoadFactor : public ControlLoop {

    void configure();
    void configure(json config);
    float step(float load_factor_command, const KinematicState& state);
    void reset(float load_factor_command, const KinematicState& state);

};

}
