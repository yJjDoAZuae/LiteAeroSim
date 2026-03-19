#define _USE_MATH_DEFINES
#include <cmath>
#include <stdexcept>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterSS2Clip.hpp"
#include <unsupported/Eigen/MatrixFunctions>  // for Matrix::pow() in controlGrammian/observeGrammian

static constexpr float kDcTol = 1e-6f;

using namespace liteaerosim::control;
using namespace liteaerosim;

FilterSS2Clip::FilterSS2Clip() {
    phi_.setZero();
    gamma_.setZero();
    h_.setZero();
    j_.setOnes();
    x_.setZero();
}

void FilterSS2Clip::copy(const FilterSS2Clip& filt) {
    phi_        = filt.phi_;
    gamma_      = filt.gamma_;
    h_          = filt.h_;
    j_          = filt.j_;
    x_          = filt.x_;
    in_         = filt.in_;
    out_        = filt.out_;
    error_code_ = filt.error_code_;
    order_      = filt.order_;
    dt_s_       = filt.dt_s_;
}

void FilterSS2Clip::setLowPassFirstIIR(float dt_s, float tau) {
    Eigen::Vector3f num_s, den_s;
    num_s << 0.0f, 0.0f, 1.0f / tau;
    den_s << 0.0f, 1.0f, 1.0f / tau;

    Eigen::Vector3f num_z, den_z;
    error_code_ += static_cast<uint16_t>(tustin_1_tf(num_s, den_s, dt_s, 2.0f * M_PI / tau, num_z, den_z));

    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 1;
    dt_s_  = dt_s;
}

void FilterSS2Clip::setLowPassSecondIIR(float dt_s, float wn_rad_s, float zeta, float tau_zero) {
    Eigen::Vector3f num_s, den_s;
    num_s << 0.0f, tau_zero * wn_rad_s * wn_rad_s, wn_rad_s * wn_rad_s;
    den_s << 1.0f, 2.0f * zeta * wn_rad_s, wn_rad_s * wn_rad_s;

    Eigen::Vector3f num_z, den_z;
    error_code_ += static_cast<uint16_t>(tustin_2_tf(num_s, den_s, dt_s, wn_rad_s, num_z, den_z));

    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 2;
    dt_s_  = dt_s;
}

void FilterSS2Clip::setNotchSecondIIR(float dt_s, float wn_rad_s, float zeta_den, float zeta_num) {
    Eigen::Vector3f num_s, den_s;
    num_s << 1.0f, 2.0f * zeta_num * wn_rad_s, wn_rad_s * wn_rad_s;
    den_s << 1.0f, 2.0f * zeta_den * wn_rad_s, wn_rad_s * wn_rad_s;

    Eigen::Vector3f num_z, den_z;
    error_code_ += static_cast<uint16_t>(tustin_2_tf(num_s, den_s, dt_s, wn_rad_s, num_z, den_z));

    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 2;
    dt_s_  = dt_s;
}

void FilterSS2Clip::setHighPassFirstIIR(float dt_s, float tau) {
    Eigen::Vector3f num_s, den_s;
    num_s << 0.0f, 1.0f / tau, 0.0f;
    den_s << 0.0f, 1.0f, 1.0f / tau;

    Eigen::Vector3f num_z, den_z;
    error_code_ = static_cast<uint16_t>(tustin_2_tf(num_s, den_s, dt_s, 2.0f * M_PI / tau, num_z, den_z));

    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 1;
    dt_s_  = dt_s;
}

void FilterSS2Clip::setHighPassSecondIIR(float dt_s, float wn_rad_s, float zeta, float c_zero) {
    Eigen::Vector3f num_s, den_s;
    num_s << 1.0f, c_zero * 2.0f * zeta * wn_rad_s, 0.0f;
    den_s << 1.0f, 2.0f * zeta * wn_rad_s, wn_rad_s * wn_rad_s;

    Eigen::Vector3f num_z, den_z;
    error_code_ += static_cast<uint16_t>(tustin_2_tf(num_s, den_s, dt_s, wn_rad_s, num_z, den_z));

    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 2;
    dt_s_  = dt_s;
}

void FilterSS2Clip::setDerivIIR(float dt_s, float tau) {
    Eigen::Vector3f num_s, den_s;
    num_s << 0.0f, 1.0f / tau, 0.0f;
    den_s << 0.0f, 1.0f, 1.0f / tau;

    Eigen::Vector3f num_z, den_z;
    error_code_ += static_cast<uint16_t>(tustin_2_tf(num_s, den_s, dt_s, 2.0f * M_PI / tau, num_z, den_z));

    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 1;
    dt_s_  = dt_s;
}

void FilterSS2Clip::resetToInput(float in_val) {
    x_.setZero();
    out_ = 0.0f;
    in_  = 0.0f;

    if (error_code_ != 0) return;

    Mat22 ImPhiInv;
    bool  invertible              = false;
    float absDeterminantThreshold = 1e-4f;

    (Mat22::Identity() - phi_).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

    if (!invertible) {
        error_code_ += static_cast<uint16_t>(FilterError::Unstable);
        return;
    }

    float dc = dcGain();
    in_  = in_val;
    out_ = valLimit.step(dc * in_val);

    if (std::fabs(dc) > kDcTol) {
        in_ = out_ / dc;
    }

    x_ = ImPhiInv * gamma_ * in_;
}

void FilterSS2Clip::resetToOutput(float out_val) {
    x_.setZero();
    out_ = 0.0f;
    in_  = 0.0f;

    float dc = dcGain();
    if (std::fabs(dc) < kDcTol) {
        error_code_ += static_cast<uint16_t>(FilterError::ZeroDcGain);
        return;
    }

    if (error_code_ != 0) return;

    Mat22 ImPhiInv;
    bool  invertible              = false;
    float absDeterminantThreshold = 1e-4f;

    (Mat22::Identity() - phi_).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

    if (!invertible) {
        error_code_ += static_cast<uint16_t>(FilterError::Unstable);
        return;
    }

    rateLimit.step(0.0f);
    out_ = valLimit.step(out_val);
    in_  = out_val / dc;
    x_   = ImPhiInv * gamma_ * in_;
}

float FilterSS2Clip::dcGain() const {
    Mat22 ImPhiInv;
    bool  invertible              = false;
    float absDeterminantThreshold = 1e-4f;

    (Mat22::Identity() - phi_).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

    if (!invertible) return 1.0f;

    return (h_ * ImPhiInv * gamma_ + j_).value();
}

Mat22 FilterSS2Clip::controlGrammian() const {
    Mat22 C(Mat22::Zero(2, 2));
    for (int k = 0; k < 2; k++) {
        C(Eigen::all, k) << Mat21(phi_.pow(k) * gamma_);
    }
    return C;
}

Mat22 FilterSS2Clip::observeGrammian() const {
    Mat22 C(Mat22::Zero(2, 2));
    for (int k = 0; k < 2; k++) {
        C(k, Eigen::all) << Mat12(h_ * phi_.pow(k));
    }
    return C;
}

void FilterSS2Clip::resetState(const Mat21& x) {
    x_  = x;
    in_ = 0.0f;
    rateLimit.step(0.0f);
    out_ = valLimit.step((h_ * x_ + j_ * in_).value());
}

void FilterSS2Clip::backsolve1(float /*in_prev*/, float /*out_prev*/, float in_curr, float out_curr) {
    bool        invertible              = false;
    const float absDeterminantThreshold = 1e-12f;

    // y = H x + J u  =>  H x = y - J u  (underdetermined for order-1)
    // Use right pseudoinverse: x = H^T (H H^T)^-1 b
    Mat12 A;
    Mat11 b;
    A << h_;
    b << out_curr - (j_ * in_curr).value();

    Mat21 ApInv;
    Mat11 AATInv;
    Mat11 AAT = A * A.transpose();
    AAT.computeInverseWithCheck(AATInv, invertible, absDeterminantThreshold);
    if (invertible) {
        ApInv = A.transpose() * AATInv;
        x_    = ApInv * b;
    }
}

void FilterSS2Clip::backsolve2(float in_prev, float out_prev, float in_curr, float out_curr) {
    Mat22 PhiInv;
    bool        invertible              = false;
    const float absDeterminantThreshold = 1e-12f;

    phi_.computeInverseWithCheck(PhiInv, invertible, absDeterminantThreshold);
    if (!invertible) return;

    // Two-equation system:
    //   y_k   = H Phi^-1 x_k - H Phi^-1 Gamma u_{k-1} + J u_{k-1}
    //   y_k+1 = H x_k + J u_k
    Mat22 A;
    Mat21 b;
    A << h_ * PhiInv,
         h_;
    b << out_prev + (h_ * PhiInv * gamma_ * in_prev).value() - (j_ * in_prev).value(),
         out_curr - (j_ * in_curr).value();

    Mat22 AInv;
    A.computeInverseWithCheck(AInv, invertible, absDeterminantThreshold);
    if (invertible) {
        x_ = AInv * b;
    } else {
        // Use left pseudoinverse: x = (A^T A)^-1 A^T b
        Mat22 ATAInv;
        (A.transpose() * A).computeInverseWithCheck(ATAInv, invertible, absDeterminantThreshold);
        if (invertible) {
            x_ = ATAInv * A.transpose() * b;
        }
    }
}

void FilterSS2Clip::backsolve(float in_prev, float out_prev, float in_curr, float out_curr) {
    if (order_ == 1) {
        backsolve1(in_prev, out_prev, in_curr, out_curr);
    } else if (order_ == 2) {
        backsolve2(in_prev, out_prev, in_curr, out_curr);
    }
}

float FilterSS2Clip::onStep(float u) {
    float in_prev  = in_;   // in_ holds the previous input during onStep
    float out_prev = out_;  // out_ holds the previous output during onStep

    // Compute filter output (before limiting)
    float out_new = (h_ * x_ + j_ * u).value();

    // Rate-limit the output change
    float out_dot     = rateLimit.step((out_new - out_prev) / dt_s_);
    float out_limited = valLimit.step(out_prev + out_dot * dt_s_);

    if (valLimit.isLimited() || rateLimit.isLimited()) {
        backsolve(in_prev, out_prev, u, out_limited);
    }

    // State update
    x_ = phi_ * x_ + gamma_ * u;

    return out_limited;
}

void FilterSS2Clip::onInitialize(const nlohmann::json& /*config*/) {
    // FilterSS2Clip is configured via set*IIR methods, not JSON config.
}

nlohmann::json FilterSS2Clip::onSerializeJson() const {
    return {
        {"in",         in_},
        {"out",        out_},
        {"dt_s",       dt_s_},
        {"order",      order_},
        {"error_code", error_code_},
        {"state",  {{"x0", x_(0, 0)}, {"x1", x_(1, 0)}}},
        {"phi",    {{"r0c0", phi_(0, 0)}, {"r0c1", phi_(0, 1)},
                    {"r1c0", phi_(1, 0)}, {"r1c1", phi_(1, 1)}}},
        {"gamma",  {{"r0", gamma_(0, 0)}, {"r1", gamma_(1, 0)}}},
        {"h",      {{"c0", h_(0, 0)},     {"c1", h_(0, 1)}}},
        {"j",      j_(0, 0)}
    };
}

void FilterSS2Clip::onDeserializeJson(const nlohmann::json& state) {
    in_         = state.at("in").get<float>();
    out_        = state.at("out").get<float>();
    dt_s_       = state.at("dt_s").get<float>();
    order_      = state.at("order").get<uint8_t>();
    error_code_ = state.at("error_code").get<uint16_t>();

    const nlohmann::json& s = state.at("state");
    x_(0, 0) = s.at("x0").get<float>();
    x_(1, 0) = s.at("x1").get<float>();

    const nlohmann::json& p = state.at("phi");
    phi_(0, 0) = p.at("r0c0").get<float>();
    phi_(0, 1) = p.at("r0c1").get<float>();
    phi_(1, 0) = p.at("r1c0").get<float>();
    phi_(1, 1) = p.at("r1c1").get<float>();

    const nlohmann::json& g = state.at("gamma");
    gamma_(0, 0) = g.at("r0").get<float>();
    gamma_(1, 0) = g.at("r1").get<float>();

    const nlohmann::json& hv = state.at("h");
    h_(0, 0) = hv.at("c0").get<float>();
    h_(0, 1) = hv.at("c1").get<float>();

    j_(0, 0) = state.at("j").get<float>();
}
