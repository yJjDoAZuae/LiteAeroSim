#define _USE_MATH_DEFINES

#include <cmath>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterTF2.hpp"

using namespace liteaerosim::control;
using namespace liteaerosim;

FilterTF2::FilterTF2() {
    num_ << 1, 0, 0;
    den_ << 1, 0, 0;
    u_buff_.setZero();
    y_buff_.setZero();
}

FilterTF2::FilterTF2(const FilterTF2& filt) {
    copy(filt);
}

void FilterTF2::copy(const FilterTF2& filt) {
    num_        = filt.num_;
    den_        = filt.den_;
    u_buff_     = filt.u_buff_;
    y_buff_     = filt.y_buff_;
    order_      = filt.order_;
    error_code_ = filt.error_code_;
}

void FilterTF2::setLowPassFirstIIR(float dt, float tau) {
    Vec3 num_s, den_s;
    num_s << 0.0f, 0.0f, 1.0f / tau;
    den_s << 0.0f, 1.0f, 1.0f / tau;

    error_code_ += static_cast<uint16_t>(tustin_1_tf(num_s, den_s, dt, 2.0f * M_PI / tau, num_, den_));
    order_ = 1;
}

void FilterTF2::setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero) {
    Vec3 num_s, den_s;
    num_s << 0.0f, tau_zero * wn_rps * wn_rps, wn_rps * wn_rps;
    den_s << 1.0f, 2.0f * zeta * wn_rps, wn_rps * wn_rps;

    error_code_ += static_cast<uint16_t>(tustin_2_tf(num_s, den_s, dt, wn_rps, num_, den_));
    order_ = 2;
}

void FilterTF2::setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num) {
    Vec3 num_s, den_s;
    num_s << 1.0f, 2.0f * zeta_num * wn_rps, wn_rps * wn_rps;
    den_s << 1.0f, 2.0f * zeta_den * wn_rps, wn_rps * wn_rps;

    error_code_ += static_cast<uint16_t>(tustin_2_tf(num_s, den_s, dt, wn_rps, num_, den_));
    order_ = 2;
}

void FilterTF2::setHighPassFirstIIR(float dt, float tau) {
    Vec3 num_s, den_s;
    num_s << 0.0f, 1.0f / tau, 0.0f;
    den_s << 0.0f, 1.0f, 1.0f / tau;

    error_code_ = static_cast<uint16_t>(tustin_2_tf(num_s, den_s, dt, 2.0f * M_PI / tau, num_, den_));
    order_ = 1;
}

void FilterTF2::setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero) {
    Vec3 num_s, den_s;
    num_s << 1.0f, c_zero * 2.0f * zeta * wn_rps, 0.0f;
    den_s << 1.0f, 2.0f * zeta * wn_rps, wn_rps * wn_rps;

    error_code_ += static_cast<uint16_t>(tustin_2_tf(num_s, den_s, dt, wn_rps, num_, den_));
    order_ = 2;
}

void FilterTF2::setDerivIIR(float dt, float tau) {
    Vec3 num_s, den_s;
    num_s << 0.0f, 1.0f / tau, 0.0f;
    den_s << 0.0f, 1.0f, 1.0f / tau;

    error_code_ += static_cast<uint16_t>(tustin_2_tf(num_s, den_s, dt, 2.0f * M_PI / tau, num_, den_));
    order_ = 1;
}

void FilterTF2::resetToInput(float in_val) {
    u_buff_.setZero();
    y_buff_.setZero();

    float dc = dcGain();

    if (error_code_ == 0) {
        u_buff_ = Vec3::Ones() * in_val;
        y_buff_ = Vec3::Ones() * in_val * dc;
    }

    in_  = u_buff_(0);
    out_ = y_buff_(0);
}

void FilterTF2::resetToOutput(float out_val) {
    u_buff_.setZero();
    y_buff_.setZero();

    float dc = dcGain();

    if (error_code_ == 0) {
        u_buff_ = Vec3::Ones() * out_val / dc;
        y_buff_ = Vec3::Ones() * out_val;
    }

    in_  = u_buff_(0);
    out_ = y_buff_(0);
}

float FilterTF2::dcGain() const {
    const float tol = 1e-6f;

    float num_sum = num_.sum();
    float den_sum = den_.sum();

    if (std::fabs(den_sum) < tol) {
        return 1.0f;
    }

    return num_sum / den_sum;
}

float FilterTF2::onStep(float u) {
    float y = num_(0) * u;

    for (int k = 1; k < order_ + 1 && k < 3; k++) {
        y += num_(k) * u_buff_(k - 1);
        y -= den_(k) * y_buff_(k - 1);
    }

    roll_buffer(y_buff_, y);
    roll_buffer(u_buff_, u);

    return y;
}

void FilterTF2::onInitialize(const nlohmann::json& /*config*/) {}

nlohmann::json FilterTF2::onSerializeJson() const {
    return {
        {"order",      order_},
        {"error_code", error_code_},
        {"in",         in_},
        {"out",        out_},
        {"num",        {num_(0), num_(1), num_(2)}},
        {"den",        {den_(0), den_(1), den_(2)}},
        {"state", {
            {"u0", u_buff_(0)}, {"u1", u_buff_(1)}, {"u2", u_buff_(2)},
            {"y0", y_buff_(0)}, {"y1", y_buff_(1)}, {"y2", y_buff_(2)}
        }}
    };
}

void FilterTF2::onDeserializeJson(const nlohmann::json& state) {
    order_      = state.at("order").get<uint8_t>();
    error_code_ = state.at("error_code").get<uint16_t>();
    in_         = state.at("in").get<float>();
    out_        = state.at("out").get<float>();

    const nlohmann::json& n = state.at("num");
    num_ << n[0].get<float>(), n[1].get<float>(), n[2].get<float>();

    const nlohmann::json& d = state.at("den");
    den_ << d[0].get<float>(), d[1].get<float>(), d[2].get<float>();

    const nlohmann::json& s = state.at("state");
    u_buff_ << s.at("u0").get<float>(), s.at("u1").get<float>(), s.at("u2").get<float>();
    y_buff_ << s.at("y0").get<float>(), s.at("y1").get<float>(), s.at("y2").get<float>();
}
