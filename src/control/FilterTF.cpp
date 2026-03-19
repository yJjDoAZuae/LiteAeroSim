#include <cmath>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterTF.hpp"

using namespace liteaerosim::control;
using namespace liteaerosim;

FilterTF::FilterTF() {
    num_.resize(1);
    den_.resize(1);
    u_buff_.resize(1);
    y_buff_.resize(1);
    num_ << 1;
    den_ << 1;
    u_buff_ << 0;
    y_buff_ << 0;
}

FilterTF::FilterTF(const FilterTF& filt) {
    copy(filt);
}

void FilterTF::copy(const FilterTF& filt) {
    int n = (maxNumStates >= filt.order()) ? filt.order() : maxNumStates;

    den_    = filt.den_.head(n + 1);
    num_    = filt.num_.head(n + 1);
    u_buff_ = filt.u_buff_.head(n + 1);
    y_buff_ = filt.y_buff_.head(n + 1);

    den_(0)     = 1.0f;
    error_code_ = filt.error_code_;
}

void FilterTF::setButterworthIIR(char order, float dt, float wn_rps) {
    if (order > 10 || order > maxNumStates) {
        return;
    }

    FiltVectorXf num_s;
    FiltVectorXf den_s;

    error_code_ += static_cast<uint16_t>(butter(order, wn_rps, num_s, den_s));

    if (error_code_ != 0) {
        return;
    }

    error_code_ += static_cast<uint16_t>(tustin_n_tf(num_s, den_s, dt, wn_rps, num_, den_));

    u_buff_.resize(order + 1);
    y_buff_.resize(order + 1);
    u_buff_.setZero();
    y_buff_.setZero();
}

void FilterTF::resetToInput(float in_val) {
    u_buff_.resize(order() + 1);
    y_buff_.resize(order() + 1);
    u_buff_.setZero();
    y_buff_.setZero();

    float dc = dcGain();

    if (error_code_ == 0) {
        u_buff_ = FiltVectorXf::Ones(order() + 1) * in_val;
        y_buff_ = FiltVectorXf::Ones(order() + 1) * in_val * dc;
    }

    in_  = u_buff_(0);
    out_ = y_buff_(0);
}

void FilterTF::resetToOutput(float out_val) {
    u_buff_.resize(order() + 1);
    y_buff_.resize(order() + 1);
    u_buff_.setZero();
    y_buff_.setZero();

    float dc = dcGain();

    if (error_code_ == 0) {
        u_buff_ = FiltVectorXf::Ones(order() + 1) * out_val / dc;
        y_buff_ = FiltVectorXf::Ones(order() + 1) * out_val;
    }

    in_  = u_buff_(0);
    out_ = y_buff_(0);
}

float FilterTF::dcGain() const {
    const float tol = 1e-6f;

    float num_sum = num_.sum();
    float den_sum = den_.sum();

    if (std::fabs(den_sum) < tol) {
        return 1.0f;
    }

    return num_sum / den_sum;
}

float FilterTF::onStep(float u) {
    float y = num_(0) * u;

    for (int k = 1; k < order() + 1 && k < kFilterMaxStates + 1; k++) {
        y += num_(k) * u_buff_(k - 1);
        y -= den_(k) * y_buff_(k - 1);
    }

    roll_buffer(y_buff_, y);
    roll_buffer(u_buff_, u);

    return y;
}

void FilterTF::onInitialize(const nlohmann::json& /*config*/) {}

nlohmann::json FilterTF::onSerializeJson() const {
    int n = order();
    auto num_arr = nlohmann::json::array();
    auto den_arr = nlohmann::json::array();
    auto u_arr   = nlohmann::json::array();
    auto y_arr   = nlohmann::json::array();
    for (int k = 0; k <= n; k++) {
        num_arr.push_back(num_(k));
        den_arr.push_back(den_(k));
        u_arr.push_back(u_buff_(k));
        y_arr.push_back(y_buff_(k));
    }
    return {
        {"order",      n},
        {"error_code", error_code_},
        {"in",         in_},
        {"out",        out_},
        {"num",        num_arr},
        {"den",        den_arr},
        {"state",      {{"u", u_arr}, {"y", y_arr}}}
    };
}

void FilterTF::onDeserializeJson(const nlohmann::json& state) {
    error_code_ = state.at("error_code").get<uint16_t>();
    in_         = state.at("in").get<float>();
    out_        = state.at("out").get<float>();

    const nlohmann::json& na = state.at("num");
    int n = static_cast<int>(na.size());
    num_.resize(n);
    for (int k = 0; k < n; k++) num_(k) = na[k].get<float>();

    const nlohmann::json& da = state.at("den");
    den_.resize(n);
    for (int k = 0; k < n; k++) den_(k) = da[k].get<float>();

    const nlohmann::json& ua = state.at("state").at("u");
    u_buff_.resize(n);
    for (int k = 0; k < n; k++) u_buff_(k) = ua[k].get<float>();

    const nlohmann::json& ya = state.at("state").at("y");
    y_buff_.resize(n);
    for (int k = 0; k < n; k++) y_buff_(k) = ya[k].get<float>();
}
