#include "Aircraft.hpp"
#include "navigation/WGS84.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace liteaerosim {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Aircraft::Aircraft(std::unique_ptr<propulsion::V_Propulsion> propulsion)
    : _propulsion(std::move(propulsion)) {}

// ---------------------------------------------------------------------------
// initialize()
// ---------------------------------------------------------------------------

void Aircraft::initialize(const nlohmann::json& config) {
    // 1. Inertia — read directly from config section (no schema_version in aircraft_config_v1)
    const auto& in_sec = config.at("inertia");
    _inertia.mass_kg   = in_sec.at("mass_kg").get<float>();
    _inertia.Ixx_kgm2  = in_sec.at("Ixx_kgm2").get<float>();
    _inertia.Iyy_kgm2  = in_sec.at("Iyy_kgm2").get<float>();
    _inertia.Izz_kgm2  = in_sec.at("Izz_kgm2").get<float>();

    // 2. Airframe performance — same pattern
    const auto& af_sec    = config.at("airframe");
    _airframe.g_max_nd    = af_sec.at("g_max_nd").get<float>();
    _airframe.g_min_nd    = af_sec.at("g_min_nd").get<float>();
    _airframe.tas_max_mps = af_sec.at("tas_max_mps").get<float>();
    _airframe.mach_max_nd = af_sec.at("mach_max_nd").get<float>();

    // 3. Lift curve
    const auto& lc = config.at("lift_curve");
    LiftCurveParams lcp{};
    lcp.cl_alpha              = lc.at("cl_alpha").get<float>();
    lcp.cl_max                = lc.at("cl_max").get<float>();
    lcp.cl_min                = lc.at("cl_min").get<float>();
    lcp.delta_alpha_stall     = lc.at("delta_alpha_stall").get<float>();
    lcp.delta_alpha_stall_neg = lc.at("delta_alpha_stall_neg").get<float>();
    lcp.cl_sep                = lc.at("cl_sep").get<float>();
    lcp.cl_sep_neg            = lc.at("cl_sep_neg").get<float>();
    _liftCurve.emplace(lcp);

    // 4. Aerodynamic performance
    const auto& ac = config.at("aircraft");
    const float S_ref_m2  = ac.at("S_ref_m2").get<float>();
    const float ar        = ac.at("ar").get<float>();
    const float e         = ac.at("e").get<float>();
    const float cd0       = ac.at("cd0").get<float>();
    const float cl_y_beta = ac.at("cl_y_beta").get<float>();
    _aeroPerf.emplace(S_ref_m2, ar, e, cd0, cl_y_beta);

    // 5. Load factor allocator (references _liftCurve — must be emplaced after step 3)
    _allocator.emplace(*_liftCurve, S_ref_m2, cl_y_beta);

    // 6. Initial kinematic state from initial_state section
    const auto& is = config.at("initial_state");
    WGS84_Datum datum;
    datum.setLatitudeGeodetic_rad(is.at("latitude_rad").get<double>());
    datum.setLongitude_rad(is.at("longitude_rad").get<double>());
    datum.setHeight_WGS84_m(is.at("altitude_m").get<float>());

    const Eigen::Vector3f vel_NED{
        is.at("velocity_north_mps").get<float>(),
        is.at("velocity_east_mps").get<float>(),
        is.at("velocity_down_mps").get<float>()
    };

    _state = KinematicState(
        0.0,
        datum,
        vel_NED,
        Eigen::Vector3f::Zero(),          // acceleration_NED_mps
        Eigen::Quaternionf::Identity(),   // q_nb — level, heading north
        Eigen::Vector3f::Zero()           // rates_Body_rps
    );
    _initial_state = _state;
}

// ---------------------------------------------------------------------------
// reset()
// ---------------------------------------------------------------------------

void Aircraft::reset() {
    _state = _initial_state;
    _allocator->reset();
    _propulsion->reset();
}

// ---------------------------------------------------------------------------
// step()
// ---------------------------------------------------------------------------

void Aircraft::step(double time_sec,
                    const AircraftCommand& cmd,
                    const Eigen::Vector3f& wind_NED_mps,
                    float rho_kgm3) {
    // 1. True airspeed
    const float V_air = (_state.velocity_NED_mps() - wind_NED_mps).norm();

    // 2. Dynamic pressure
    const float q_inf = 0.5f * rho_kgm3 * V_air * V_air;

    // 3. Clamp load factors to airframe structural limits
    const float n_cmd   = std::clamp(cmd.n,   _airframe.g_min_nd, _airframe.g_max_nd);
    const float n_y_cmd = std::clamp(cmd.n_y, _airframe.g_min_nd, _airframe.g_max_nd);

    // 4. Solve for α and β
    LoadFactorInputs lfa_in;
    lfa_in.n        = n_cmd;
    lfa_in.n_y      = n_y_cmd;
    lfa_in.q_inf    = q_inf;
    lfa_in.thrust_n = _propulsion->thrust_n();   // previous-step thrust (0 on first call)
    lfa_in.mass_kg  = _inertia.mass_kg;
    lfa_in.n_dot    = cmd.n_dot;
    lfa_in.n_y_dot  = cmd.n_y_dot;
    const LoadFactorOutputs lfa_out = _allocator->solve(lfa_in);

    // 5. Lift coefficient
    const float cl = _liftCurve->evaluate(lfa_out.alpha_rad);

    // 6. Aerodynamic forces in Wind frame
    const aerodynamics::AeroForces F =
        _aeroPerf->compute(lfa_out.alpha_rad, lfa_out.beta_rad, q_inf, cl);

    // 7. Advance propulsion
    const float T = _propulsion->step(cmd.throttle_nd, V_air, rho_kgm3);

    // 8. Wind-frame acceleration.
    //    Thrust decomposition (Wind frame, X forward, Y right, Z down):
    //      Tx =  T·cos(α)·cos(β)
    //      Ty = -T·cos(α)·sin(β)
    //      Tz = -T·sin(α)
    //    Gravity is embedded in the load-factor constraint — must not be added here.
    const float m  = _inertia.mass_kg;
    const float ca = std::cos(lfa_out.alpha_rad);
    const float sa = std::sin(lfa_out.alpha_rad);
    const float cb = std::cos(lfa_out.beta_rad);
    const float sb = std::sin(lfa_out.beta_rad);

    const float ax = (T * ca * cb + F.x_n) / m;
    const float ay = (-T * ca * sb + F.y_n) / m;
    const float az = (-T * sa      + F.z_n) / m;

    // 9. Advance kinematic state
    _state.step(time_sec,
                Eigen::Vector3f{ax, ay, az},
                cmd.rollRate_Wind_rps,
                lfa_out.alpha_rad,
                lfa_out.beta_rad,
                lfa_out.alphaDot_rps,
                lfa_out.betaDot_rps,
                wind_NED_mps);
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

nlohmann::json Aircraft::serializeJson() const {
    nlohmann::json j;
    j["schema_version"]  = 1;
    j["type"]            = "Aircraft";
    j["kinematic_state"] = _state.serializeJson();
    j["initial_state"]   = _initial_state.serializeJson();
    j["airframe"]        = _airframe.serializeJson();
    j["inertia"]         = _inertia.serializeJson();
    if (_liftCurve)  j["lift_curve"]      = _liftCurve->serializeJson();
    if (_aeroPerf)   j["aero_performance"] = _aeroPerf->serializeJson();
    if (_allocator)  j["allocator"]        = _allocator->serializeJson();
    if (_propulsion) j["propulsion"]       = _propulsion->serializeJson();
    return j;
}

void Aircraft::deserializeJson(const nlohmann::json& j) {
    _airframe = AirframePerformance::deserializeJson(j.at("airframe"));
    _inertia  = Inertia::deserializeJson(j.at("inertia"));

    _liftCurve.emplace(LiftCurveModel::deserializeJson(j.at("lift_curve")));
    _aeroPerf.emplace(aerodynamics::AeroPerformance::deserializeJson(j.at("aero_performance")));

    // Emplace allocator with placeholder config — deserializeJson overwrites _S and _cl_y_beta.
    // The reference to _liftCurve is set at construction and is the only field not restored.
    _allocator.emplace(*_liftCurve, 1.0f, -0.1f);
    _allocator->deserializeJson(j.at("allocator"));

    _state.deserializeJson(j.at("kinematic_state"));
    _initial_state.deserializeJson(j.at("initial_state"));

    if (_propulsion) {
        _propulsion->deserializeJson(j.at("propulsion"));
    }
}

} // namespace liteaerosim
