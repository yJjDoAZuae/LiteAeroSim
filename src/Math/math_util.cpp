#define _USE_MATH_DEFINES

#include "Math/math_util.hpp"
#include <cmath>

using namespace MathUtil;

double MathUtil::wrapToPi(double angle_rad)
{
    double wrapped_angle = fmod(angle_rad + M_PI, 2.0 * M_PI);
    if (wrapped_angle < 0)
    {
        wrapped_angle += 2.0 * M_PI;
    }
    return wrapped_angle - M_PI;
}

