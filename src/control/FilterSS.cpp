#define _USE_MATH_DEFINES
#include <cmath>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterSS.hpp"

using namespace liteaerosim::control;
using namespace liteaerosim;

FilterSS::FilterSS() {
    phi_.resize(0, 0);
    gamma_.resize(0, 1);
    h_.resize(1, 0);
    j_.setOnes();
    x_.resize(0, 1);
}

FilterSS::FilterSS(const FilterSS& filt) {
    copy(filt);
}

FilterSS::~FilterSS() {}

void FilterSS::copy(const FilterSS& filt) {
    phi_        = filt.phi_;
    gamma_      = filt.gamma_;
    h_          = filt.h_;
    j_          = filt.j_;
    x_          = filt.x_;
    error_code_ = filt.error_code_;
}

void FilterSS::setButterworthIIR(uint8_t order, float dt, float wn_rps) {
    FiltVectorXf num_s;
    FiltVectorXf den_s;

    FilterError rc = butter(order, wn_rps, num_s, den_s);

    if (rc != FilterError::None) {
        error_code_ += static_cast<uint16_t>(rc);
        return;
    }

    MatNN A;
    MatN1 B;
    Mat1N C;
    Mat11 D;

    tf2ss(num_s, den_s, A, B, C, D);

    setDimension(order);

    error_code_ += static_cast<uint16_t>(tustin_n_ss(A, B, C, D, dt, wn_rps, phi_, gamma_, h_, j_));

    resetToInput(0.0f);
}

void FilterSS::resetToInput(float in_val) {
    x_.setZero();
    out_ = 0.0f;
    in_  = 0.0f;

    float dc = dcGain();

    if (error_code_ == 0) {
        int n = phi_.rows();

        MatNN ImPhi(MatNN::Identity(n, n) - phi_);
        Eigen::FullPivLU<MatNN> LU(ImPhi);

        if (!LU.isInvertible()) {
            error_code_ += static_cast<uint16_t>(FilterError::Unstable);
            return;
        }

        in_  = in_val;
        out_ = dc * in_val;
        x_   = LU.inverse() * gamma_ * in_;
    }
}

void FilterSS::resetToOutput(float out_val) {
    x_.setZero();
    out_ = 0.0f;
    in_  = 0.0f;

    float dc = dcGain();

    if (error_code_ == 0) {
        int n = phi_.rows();

        MatNN ImPhi(MatNN::Identity(n, n) - phi_);
        Eigen::FullPivLU<MatNN> LU(ImPhi);

        if (!LU.isInvertible()) {
            error_code_ += static_cast<uint16_t>(FilterError::Unstable);
            return;
        }

        out_ = out_val;
        in_  = out_val / dc;
        x_   = LU.inverse() * gamma_ * in_;
    }
}

float FilterSS::dcGain() const {
    int n = phi_.rows();

    MatNN ImPhi(MatNN::Identity(n, n) - phi_);
    Eigen::FullPivLU<MatNN> LU(ImPhi);

    if (!LU.isInvertible()) {
        return 1.0f;
    }

    return (h_ * LU.inverse() * gamma_ + j_).value();
}

MatNN FilterSS::controlGrammian() const {
    int n = phi_.rows();
    MatNN C(MatNN::Zero(n, n));
    MatN1 Phi_k_Gamma = gamma_;
    for (int k = 0; k < n; k++) {
        C.col(k) = Phi_k_Gamma;
        Phi_k_Gamma = phi_ * Phi_k_Gamma;
    }
    return C;
}

MatNN FilterSS::observeGrammian() const {
    int n = phi_.rows();
    MatNN C(MatNN::Zero(n, n));
    Mat1N H_Phi_k = h_;
    for (int k = 0; k < n; k++) {
        C.row(k) = H_Phi_k;
        H_Phi_k = H_Phi_k * phi_;
    }
    return C;
}

uint8_t FilterSS::order() const {
    uint8_t n = static_cast<uint8_t>(phi_.rows());

    if (n > 0) {
        Eigen::JacobiSVD<MatNN> control_svd;
        Eigen::JacobiSVD<MatNN> observe_svd;

        MatNN CC = controlGrammian();
        MatNN CO = observeGrammian();

        control_svd.compute(CC);
        observe_svd.compute(CO);

        uint8_t crank = static_cast<uint8_t>(control_svd.rank());
        n = (crank < n) ? crank : n;
        uint8_t orank = static_cast<uint8_t>(observe_svd.rank());
        n = (orank < n) ? orank : n;
    }

    return n;
}

void FilterSS::setDimension(uint8_t dim) {
    phi_.resize(dim, dim);
    gamma_.resize(dim, 1);
    h_.resize(1, dim);
    j_.resize(1, 1);
    x_.resize(dim, 1);
}

float FilterSS::onStep(float u) {
    float y = (h_ * x_ + j_ * u).value();
    x_ = phi_ * x_ + gamma_ * u;
    return y;
}

void FilterSS::onInitialize(const nlohmann::json& /*config*/) {}

nlohmann::json FilterSS::onSerializeJson() const {
    int n = static_cast<int>(phi_.rows());
    auto phi_arr   = nlohmann::json::array();
    auto gamma_arr = nlohmann::json::array();
    auto h_arr     = nlohmann::json::array();
    auto x_arr     = nlohmann::json::array();
    for (int i = 0; i < n; i++) {
        for (int k = 0; k < n; k++) phi_arr.push_back(phi_(i, k));
        gamma_arr.push_back(gamma_(i, 0));
        h_arr.push_back(h_(0, i));
        x_arr.push_back(x_(i, 0));
    }
    return {
        {"order",      n},
        {"error_code", error_code_},
        {"in",         in_},
        {"out",        out_},
        {"j",          j_(0, 0)},
        {"phi",        phi_arr},
        {"gamma",      gamma_arr},
        {"h",          h_arr},
        {"state",      {{"x", x_arr}}}
    };
}

void FilterSS::onDeserializeJson(const nlohmann::json& state) {
    error_code_ = state.at("error_code").get<uint16_t>();
    in_         = state.at("in").get<float>();
    out_        = state.at("out").get<float>();
    j_(0, 0)    = state.at("j").get<float>();

    int n = state.at("order").get<int>();
    setDimension(static_cast<uint8_t>(n));

    const nlohmann::json& pa = state.at("phi");
    for (int i = 0; i < n; i++)
        for (int k = 0; k < n; k++)
            phi_(i, k) = pa[i * n + k].get<float>();

    const nlohmann::json& ga = state.at("gamma");
    for (int i = 0; i < n; i++) gamma_(i, 0) = ga[i].get<float>();

    const nlohmann::json& ha = state.at("h");
    for (int i = 0; i < n; i++) h_(0, i) = ha[i].get<float>();

    const nlohmann::json& xa = state.at("state").at("x");
    for (int i = 0; i < n; i++) x_(i, 0) = xa[i].get<float>();
}
