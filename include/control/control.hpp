#pragma once

#include "numerics.hpp"

namespace liteaerosim::control {

enum class FilterError : uint16_t {
    None             = 0,
    InvalidDimension = 1,
    InvalidTimestep  = 2,
    Unstable         = 4,
    InfiniteDcGain   = 8,
    ZeroDcGain       = 16,
    InvalidPolynomial = 32,
};

enum class DiscretizationMethod {
    ForwardEuler  = 0,
    BackwardEuler = 1,
    Bilinear      = 2,
    Prewarp       = 3,
    PoleZeroMatch = 4,
};

}  // namespace liteaerosim::control
