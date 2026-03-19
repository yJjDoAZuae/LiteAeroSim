#include <cmath>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterFIR.hpp"

using namespace liteaerosim::control;
using namespace liteaerosim;

FilterFIR::FilterFIR() {
    num_.resize(1);
    u_buff_.resize(1);
    num_ << 1.0f;
    u_buff_ << 0.0f;
}

FilterFIR::FilterFIR(const FilterFIR& filt) {
    copy(filt);
}

void FilterFIR::copy(const FilterFIR& filt) {
    num_        = filt.num_;
    u_buff_     = filt.u_buff_;
    error_code_ = filt.error_code_;
}

void FilterFIR::setAverageFIR(char order) {
    num_    = FiltVectorXf::Ones(order + 1) * (1.0f / (order + 1.0f));
    u_buff_ = FiltVectorXf::Zero(order + 1);
}

void FilterFIR::setExpFIR(char order, float dt, float tau) {
    num_    = FiltVectorXf::Zero(order + 1);
    u_buff_ = FiltVectorXf::Zero(order + 1);

    for (int k = 0; k < this->order() + 1; k++) {
        num_(k) = std::exp(-k * dt / tau);
    }
    float sum_num = num_.sum();
    num_ *= 1.0f / sum_num;
}

void FilterFIR::resetToInput(float in_val) {
    u_buff_.setZero();
    u_buff_ = FiltVectorXf::Ones(order() + 1) * in_val;
    in_  = u_buff_(0);
    out_ = in_ * dcGain();
}

void FilterFIR::resetToOutput(float out_val) {
    const float tol = 1e-6f;
    u_buff_.setZero();

    float dc = dcGain();

    if (std::fabs(dc) > tol) {
        u_buff_ = FiltVectorXf::Ones(order() + 1) * out_val / dc;
        in_  = u_buff_(0);
        out_ = in_ / dc;
    } else {
        error_code_ += static_cast<uint16_t>(FilterError::ZeroDcGain);
        in_  = 0.0f;
        out_ = 0.0f;
    }
}

float FilterFIR::dcGain() const {
    return num_.sum();
}

float FilterFIR::onStep(float u) {
    float y = num_(0) * u;

    for (int k = 1; k < order() + 1; k++) {
        y += num_(k) * u_buff_(k - 1);
    }

    roll_buffer(u_buff_, y);

    return y;
}

void FilterFIR::onInitialize(const nlohmann::json& /*config*/) {}

nlohmann::json FilterFIR::onSerializeJson() const {
    int n = order();
    auto num_arr = nlohmann::json::array();
    auto u_arr   = nlohmann::json::array();
    for (int k = 0; k <= n; k++) {
        num_arr.push_back(num_(k));
        u_arr.push_back(u_buff_(k));
    }
    return {
        {"order",      n},
        {"error_code", error_code_},
        {"in",         in_},
        {"out",        out_},
        {"num",        num_arr},
        {"state",      {{"u", u_arr}}}
    };
}

void FilterFIR::onDeserializeJson(const nlohmann::json& state) {
    error_code_ = state.at("error_code").get<uint16_t>();
    in_         = state.at("in").get<float>();
    out_        = state.at("out").get<float>();

    const nlohmann::json& na = state.at("num");
    int n = static_cast<int>(na.size());
    num_.resize(n);
    for (int k = 0; k < n; k++) num_(k) = na[k].get<float>();

    const nlohmann::json& ua = state.at("state").at("u");
    u_buff_.resize(n);
    for (int k = 0; k < n; k++) u_buff_(k) = ua[k].get<float>();
}
