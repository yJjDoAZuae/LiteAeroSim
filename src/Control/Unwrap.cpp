#include "control/Unwrap.hpp"
#include "Math/math_util.hpp"

using namespace Control;

float Unwrap::step(float u) 
{
    _in = u;
    _out += MathUtil::wrapToPi(_in - _out);;

    return _out;
}

float Unwrap::step(float u, float ref) 
{
    _in = u;
    _out += MathUtil::wrapToPi(_in - ref);;

    return _out;
}
