
#include "Control/control.hpp"

using namespace Control;

// resize vector to specified length by either truncating from the left or
// zero padding from the left
FiltVectorXf left_resize(const FiltVectorXf &in, char len)
{
    FiltVectorXf out(len);

    if (in.size() < len) {

        out << FiltVectorXf::Zero(len - in.size()), in;

        // // zero padding
        // out.head(len - in.size()).setZero();

        // // copy input
        // out.tail(in.size()) = in;
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

        // // zero padding
        // out.tail(len - in.size()).setZero();

        // // copy input
        // out.head(in.size()) = in;
    } else {
        out << in.head(len);
    }

    return out;
}

// resize vector to specified length by either truncating from the right or
// zero padding from the right
void roll_buffer(FiltVectorXf &buff, float u)
{

    if (buff.size() > 0) {
        buff << u, buff.head(buff.size() - 1);
    }

    // // zero padding
    // out.tail(len - in.size()).setZero();

    // // copy input
    // out.head(in.size()) = in;
}
