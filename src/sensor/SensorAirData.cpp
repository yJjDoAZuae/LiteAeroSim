#include "sensor/SensorAirData.hpp"
#include "liteaerosim.pb.h"
#include <cmath>
#include <random>
#include <stdexcept>

namespace liteaero::simulation {

// ─── ISA constants ────────────────────────────────────────────────────────────
static constexpr float kP0     = 101325.0f;   // sea-level ISA pressure (Pa)
static constexpr float kP11000 = 22632.1f;    // ISA pressure at tropopause base (Pa)
static constexpr float kRho0   = 1.225f;      // sea-level ISA density (kg/m³)
static constexpr float kA0     = 340.294f;    // sea-level ISA speed of sound (m/s)
static constexpr float kT0     = 288.15f;     // sea-level ISA temperature (K)
static constexpr float kT11000 = 216.65f;     // tropopause temperature (K)
static constexpr float kRd     = 287.058f;    // dry-air gas constant (J/(kg·K))
static constexpr float kG0     = 9.80665f;    // standard gravity (m/s²)
static constexpr float kL      = -0.0065f;    // troposphere lapse rate (K/m)
// Barometric altitude exponent: -Rd·L/g0 = Rd·|L|/g0
static constexpr float kBaroExp = 0.190263f;

// ─── RNG pimpl ───────────────────────────────────────────────────────────────
struct SensorAirData::RngState {
    uint32_t seed          = 0;
    uint64_t advance_count = 0;
    std::mt19937 engine;
    std::normal_distribution<float> dist{0.0f, 1.0f};

    void reseed(uint32_t s) {
        seed          = s;
        advance_count = 0;
        engine.seed(s);
        dist.reset();
    }

    float draw() {
        ++advance_count;
        return dist(engine);
    }
};

// ─── Lifecycle ───────────────────────────────────────────────────────────────
SensorAirData::SensorAirData(const nlohmann::json& config) {
    initialize(config);
}

SensorAirData::~SensorAirData() = default;

void SensorAirData::onInitialize(const nlohmann::json& cfg) {
    config_.differential_pressure_noise_pa  = cfg.value("differential_pressure_noise_pa",  0.0f);
    config_.static_pressure_noise_pa        = cfg.value("static_pressure_noise_pa",        0.0f);
    config_.differential_pressure_lag_tau_s = cfg.value("differential_pressure_lag_tau_s", 0.0f);
    config_.static_pressure_lag_tau_s       = cfg.value("static_pressure_lag_tau_s",       0.0f);
    config_.static_pressure_bias_pa         = cfg.value("static_pressure_bias_pa",         0.0f);
    config_.static_port_angle_rad           = cfg.value("static_port_angle_rad",           0.0f);
    config_.oat_noise_k                     = cfg.value("oat_noise_k",                     0.0f);
    config_.dt_s                            = cfg.value("dt_s",                             0.01f);
    config_.initial_kollsman_pa             = cfg.value("initial_kollsman_pa",          101325.0f);
    config_.seed = cfg.value("seed", static_cast<uint32_t>(0));

    kollsman_pa_ = config_.initial_kollsman_pa;

    if (!rng_) rng_ = std::make_unique<RngState>();
    const uint32_t seed = (config_.seed == 0)
                              ? static_cast<uint32_t>(std::random_device{}())
                              : config_.seed;
    rng_->reseed(seed);

    qc_lag_state_      = 0.0f;
    qc_lag_prev_input_ = 0.0f;
    ps_lag_state_      = 0.0f;
    ps_lag_prev_input_ = 0.0f;
    last_qc_true_      = 0.0f;
    last_ps_true_      = kP0;
}

void SensorAirData::onReset() {
    kollsman_pa_ = config_.initial_kollsman_pa;

    // Re-seed RNG from the same seed to restore the initial sequence.
    if (rng_) rng_->reseed(rng_->seed);

    // Initialize lag filter states to steady-state at the last known true values.
    // This ensures the first step() after reset() returns the noiseless zero-lag value.
    qc_lag_state_      = last_qc_true_;
    qc_lag_prev_input_ = last_qc_true_;
    ps_lag_state_      = last_ps_true_;
    ps_lag_prev_input_ = last_ps_true_;
}

// ─── Step ────────────────────────────────────────────────────────────────────
AirDataMeasurement SensorAirData::step(Eigen::Vector3f v_body,
                                        const AtmosphericState& atm) {
    const float u    = v_body.x();
    const float v    = v_body.y();
    const float w    = v_body.z();
    const float VTas = v_body.norm();

    // True impact pressure and static pressure
    const float a_sound = atm.speed_of_sound_mps;
    const float M_true  = (a_sound > 1.0f) ? VTas / a_sound : 0.0f;
    const float qc_true = atm.pressure_pa
                          * (std::pow(1.0f + 0.2f * M_true * M_true, 3.5f) - 1.0f);
    const float ps_true = atm.pressure_pa;

    // Fuselage crossflow static pressure error (two-port symmetric crosslinked model)
    float delta_p_geo = 0.0f;
    if (VTas > 1.0e-3f) {
        const float sin_alpha = w / VTas;
        const float sin_beta  = v / VTas;
        const float delta     = std::atan2(sin_beta, -sin_alpha);
        const float sin2_sum  = sin_alpha * sin_alpha + sin_beta * sin_beta;
        const float q_cross   = 0.5f * atm.density_kgm3 * VTas * VTas * sin2_sum;
        const float phi       = config_.static_port_angle_rad;
        const float cp_bar    = -1.0f - 2.0f * std::cos(2.0f * phi) * std::cos(2.0f * delta);
        delta_p_geo           = cp_bar * q_cross;
    }

    // Noise draws — fixed order per step: qc, Ps, OAT
    const float n_qc  = rng_->draw() * config_.differential_pressure_noise_pa;
    const float n_ps  = rng_->draw() * config_.static_pressure_noise_pa;
    const float n_oat = rng_->draw() * config_.oat_noise_k;

    // Noisy signals
    const float qc_noisy = qc_true + n_qc;
    const float ps_noisy = ps_true + delta_p_geo + config_.static_pressure_bias_pa + n_ps;

    // First-order Tustin lag — qc channel
    float qc_meas;
    const float dt = config_.dt_s;
    if (config_.differential_pressure_lag_tau_s > 0.0f) {
        const float tau   = config_.differential_pressure_lag_tau_s;
        const float alpha = dt / (2.0f * tau + dt);
        const float beta  = (2.0f * tau - dt) / (2.0f * tau + dt);
        qc_meas = alpha * (qc_noisy + qc_lag_prev_input_) + beta * qc_lag_state_;
    } else {
        qc_meas = qc_noisy;
    }
    qc_lag_state_      = qc_meas;
    qc_lag_prev_input_ = qc_noisy;

    // First-order Tustin lag — Ps channel
    float ps_meas;
    if (config_.static_pressure_lag_tau_s > 0.0f) {
        const float tau   = config_.static_pressure_lag_tau_s;
        const float alpha = dt / (2.0f * tau + dt);
        const float beta  = (2.0f * tau - dt) / (2.0f * tau + dt);
        ps_meas = alpha * (ps_noisy + ps_lag_prev_input_) + beta * ps_lag_state_;
    } else {
        ps_meas = ps_noisy;
    }
    ps_lag_state_      = ps_meas;
    ps_lag_prev_input_ = ps_noisy;

    // Store true values for reset() steady-state initialization
    last_qc_true_ = qc_true;
    last_ps_true_ = ps_true;

    // Clamp negative impact pressure (possible when noise > true qc at near-zero airspeed)
    if (qc_meas < 0.0f) qc_meas = 0.0f;

    // Derived quantities
    AirDataMeasurement meas{};

    if (qc_meas > 0.0f) {
        // ADC Mach: invert isentropic impact pressure equation
        meas.mach_nd = std::sqrt(5.0f * (std::pow(qc_meas / ps_meas + 1.0f, 2.0f / 7.0f) - 1.0f));

        // TAS: Mach × local speed of sound
        meas.tas_mps = meas.mach_nd * atm.speed_of_sound_mps;

        // IAS: incompressible Bernoulli at sea-level ISA density
        meas.ias_mps = std::sqrt(2.0f * qc_meas / kRho0);

        // CAS: isentropic formula referenced to sea-level ISA pressure
        meas.cas_mps = kA0 * std::sqrt(5.0f * (std::pow(qc_meas / kP0 + 1.0f, 2.0f / 7.0f) - 1.0f));

        // EAS: TAS scaled by density ratio
        meas.eas_mps = meas.tas_mps * std::sqrt(atm.density_kgm3 / kRho0);
    }

    meas.baro_altitude_m = baro_altitude(ps_meas);
    meas.oat_k           = atm.temperature_k + n_oat;

    return meas;
}

float SensorAirData::baro_altitude(float ps_meas) const {
    // Kollsman-adjusted tropopause boundary pressure
    const float p_koll_11000 = kP11000 * kollsman_pa_ / kP0;

    if (ps_meas > p_koll_11000) {
        // Troposphere: h = (T0/L) · [(Ps/PKoll)^(-Rd·L/g0) - 1]
        // where kBaroExp = -Rd·L/g0 > 0  and  T0/L < 0
        return (kT0 / kL) * (std::pow(ps_meas / kollsman_pa_, kBaroExp) - 1.0f);
    }
    // Tropopause / lower stratosphere (isothermal layer)
    return 11000.0f - (kRd * kT11000 / kG0) * std::log(ps_meas / p_koll_11000);
}

// ─── Kollsman ────────────────────────────────────────────────────────────────
void  SensorAirData::setKollsman(float pa) { kollsman_pa_ = pa; }
float SensorAirData::kollsman_pa()   const { return kollsman_pa_; }

// ─── JSON serialization ──────────────────────────────────────────────────────
nlohmann::json SensorAirData::onSerializeJson() const {
    return {
        {"qc_lag_state",      qc_lag_state_},
        {"qc_lag_prev_input", qc_lag_prev_input_},
        {"ps_lag_state",      ps_lag_state_},
        {"ps_lag_prev_input", ps_lag_prev_input_},
        {"rng_seed",          rng_->seed},
        {"rng_advance",       rng_->advance_count},
        {"kollsman_pa",       kollsman_pa_},
    };
}

void SensorAirData::onDeserializeJson(const nlohmann::json& state) {
    qc_lag_state_      = state.at("qc_lag_state").get<float>();
    qc_lag_prev_input_ = state.at("qc_lag_prev_input").get<float>();
    ps_lag_state_      = state.at("ps_lag_state").get<float>();
    ps_lag_prev_input_ = state.at("ps_lag_prev_input").get<float>();
    kollsman_pa_       = state.at("kollsman_pa").get<float>();

    const auto seed = state.at("rng_seed").get<uint32_t>();
    const auto adv  = state.at("rng_advance").get<uint64_t>();

    if (!rng_) rng_ = std::make_unique<RngState>();
    rng_->reseed(seed);
    for (uint64_t i = 0; i < adv; ++i) rng_->draw();
}

// ─── Proto serialization ─────────────────────────────────────────────────────
std::vector<uint8_t> SensorAirData::serializeProto() const {
    las_proto::AirDataStateProto proto;
    proto.set_schema_version(1);
    proto.set_qc_lag_state(qc_lag_state_);
    proto.set_qc_lag_prev_input(qc_lag_prev_input_);
    proto.set_ps_lag_state(ps_lag_state_);
    proto.set_ps_lag_prev_input(ps_lag_prev_input_);
    proto.set_rng_seed(rng_->seed);
    proto.set_rng_advance(rng_->advance_count);
    proto.set_kollsman_pa(kollsman_pa_);

    const std::string bytes = proto.SerializeAsString();
    return {bytes.begin(), bytes.end()};
}

void SensorAirData::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::AirDataStateProto proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
        throw std::runtime_error("SensorAirData::deserializeProto: parse failed");
    }
    if (proto.schema_version() != 1) {
        throw std::runtime_error(
            "SensorAirData::deserializeProto: schema version mismatch: stored=" +
            std::to_string(proto.schema_version()) + " current=1");
    }

    qc_lag_state_      = proto.qc_lag_state();
    qc_lag_prev_input_ = proto.qc_lag_prev_input();
    ps_lag_state_      = proto.ps_lag_state();
    ps_lag_prev_input_ = proto.ps_lag_prev_input();
    kollsman_pa_       = proto.kollsman_pa();

    if (!rng_) rng_ = std::make_unique<RngState>();
    rng_->reseed(proto.rng_seed());
    for (uint64_t i = 0; i < proto.rng_advance(); ++i) rng_->draw();
}

}  // namespace liteaero::simulation
