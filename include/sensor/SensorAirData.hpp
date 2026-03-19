#pragma once
#include "DynamicElement.hpp"
#include "environment/AtmosphericState.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <vector>

namespace liteaerosim::sensor {

// Output struct returned by SensorAirData::step(). All quantities are derived from
// the noisy, lagged transducer readings.
struct AirDataMeasurement {
    float ias_mps;          // indicated airspeed: raw pitot-static indication (m/s)
    float cas_mps;          // calibrated airspeed: IAS corrected for compressibility (m/s)
    float eas_mps;          // equivalent airspeed: same dyn. pressure at sea-level ISA density (m/s)
    float tas_mps;          // true airspeed: wind-relative speed at flight altitude (m/s)
    float mach_nd;          // Mach number (non-dimensional)
    float baro_altitude_m;  // indicated barometric altitude, Kollsman-referenced (m)
    float oat_k;            // outside air temperature with noise (K)
};

// Configuration struct. Supplied as a JSON object to initialize(). All fields default
// to zero (ideal, noiseless, lagless sensor) except initial_kollsman_pa and dt_s.
struct AirDataConfig {
    float    differential_pressure_noise_pa  = 0.0f;     // 1-σ noise on qc transducer (Pa)
    float    static_pressure_noise_pa        = 0.0f;     // 1-σ noise on Ps transducer (Pa)
    float    differential_pressure_lag_tau_s = 0.0f;     // 1st-order lag τ, qc channel (s)
    float    static_pressure_lag_tau_s       = 0.0f;     // 1st-order lag τ, Ps channel (s)
    float    static_pressure_bias_pa         = 0.0f;     // constant residual static port offset (Pa)
    float    static_port_angle_rad           = 0.0f;     // port angle from waterline, positive up (rad)
    float    oat_noise_k                     = 0.0f;     // 1-σ noise on OAT reading (K)
    float    dt_s                            = 0.01f;    // simulation timestep (s)
    float    initial_kollsman_pa             = 101325.0f;// initial altimeter QNH setting (Pa)
    uint32_t seed                            = 0;        // RNG seed; 0 = non-deterministic
    int      schema_version                  = 1;
};

// Pitot-static air data computer.
//
// Contains two internal transducer channels (differential pressure qc, static pressure Ps),
// each with additive Gaussian noise and a first-order Tustin-discretized lag. Derives IAS,
// CAS, EAS, TAS, Mach, barometric altitude, and OAT from the noisy lagged readings.
// The static pressure channel models the fuselage crossflow pressure error via the
// two-port symmetric crosslinked model (see docs/algorithms/air_data.md).
class SensorAirData : public liteaerosim::DynamicElement {
public:
    explicit SensorAirData(const nlohmann::json& config);
    ~SensorAirData();  // defined in .cpp — RngState pimpl requires complete type at destruction

    AirDataMeasurement step(Eigen::Vector3f airspeed_body_mps,
                            const liteaerosim::environment::AtmosphericState& atm);

    // Runtime Kollsman (QNH) update. Takes effect immediately on the next step().
    void  setKollsman(float pa);
    float kollsman_pa() const;

    [[nodiscard]] std::vector<uint8_t> serializeProto() const;
    void deserializeProto(const std::vector<uint8_t>& bytes);

protected:
    void           onInitialize(const nlohmann::json& config) override;
    void           onReset()                                  override;
    nlohmann::json onSerializeJson()                  const   override;
    void           onDeserializeJson(const nlohmann::json& state) override;
    int            schemaVersion()                    const   override { return 1; }
    const char*    typeName()                         const   override { return "SensorAirData"; }

private:
    struct RngState;                    // pimpl — hides mt19937 + normal_distribution internals

    AirDataConfig config_;
    float kollsman_pa_        = 101325.0f;
    float qc_lag_state_       = 0.0f;   // lag filter output at previous step, qc channel
    float qc_lag_prev_input_  = 0.0f;   // noisy input to lag filter at previous step, qc channel
    float ps_lag_state_       = 0.0f;   // lag filter output at previous step, Ps channel
    float ps_lag_prev_input_  = 0.0f;   // noisy input to lag filter at previous step, Ps channel
    float last_qc_true_       = 0.0f;   // true qc from last step (for reset() steady-state init)
    float last_ps_true_       = 101325.0f; // true Ps from last step (for reset() steady-state init)

    std::unique_ptr<RngState> rng_;

    // Barometric altitude inversion using current Kollsman setting.
    float baro_altitude(float ps_meas) const;
};

}  // namespace liteaerosim::sensor
