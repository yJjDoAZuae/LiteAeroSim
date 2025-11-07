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

// limit a value to between min/max
double MathUtil::clip(double u, double min, double max) {
    double i_max = fmax(min,max);

    u = (u>min)? u : min;
    return (u<i_max)? u : i_max;
}
