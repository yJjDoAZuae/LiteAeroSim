
#include "control/ControlLoop.hpp"
#include "KinematicState.hpp"

namespace liteaero::simulation {

class ControlAltitude : public ControlLoop {

    void configure();
    void configure(json config);
    float step(float command, const KinematicState& state);
    void reset(float command, const KinematicState& state);

};

}
