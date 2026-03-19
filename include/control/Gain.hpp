#pragma once

#include <cstdint>

namespace liteaerosim::control {

/// Simple gain element with template parameters reserved for future scheduling design.
///
/// Template parameters T (value type) and NumAxes (scheduling axis count) are retained
/// to preserve API compatibility for the planned gain scheduling implementation.
/// The scheduling logic itself is not yet designed; see roadmap item 3.
template <typename T, uint32_t NumAxes>
class Gain {
public:
    Gain() : value_(0) {}
    explicit Gain(T value) : value_(value) {}

    T    value() const      { return value_; }
    void set(T value)       { value_ = value; }

    operator float() const  { return static_cast<float>(value_); }

private:
    T value_;
};

}  // namespace liteaerosim::control
