
#include "control/control.hpp"

using namespace Control;

namespace Control {

// resize vector to specified length by either truncating from the left or
// zero padding from the left
FiltVectorXf left_resize(const FiltVectorXf &in, char len)
{
    FiltVectorXf out(len);

    if (in.size() < len) {
        out << FiltVectorXf::Zero(len - in.size()), in;
    } else {
        out << in.tail(len);
    }

    return out;
}

// resize vector to specified length by either truncating from the right or
// zero padding from the right
FiltVectorXf right_resize(const FiltVectorXf &in, char len)
{
    FiltVectorXf out(len);

    if (in.size() < len) {
        out << in, FiltVectorXf::Zero(len - in.size());
    } else {
        out << in.head(len);
    }

    return out;
}

// roll a data buffer to the right and update the first entry with new data
void roll_buffer(FiltVectorXf &buff, float u)
{
    if (buff.size() > 0) {
        buff << u, buff.head(buff.size() - 1);
    }
}

// roll a data buffer to the right and update the first entry with new data
void roll_buffer(Vec3 &buff, float u)
{
    buff(2) = buff(1);
    buff(1) = buff(0);
    buff(0) = u;
}

}
