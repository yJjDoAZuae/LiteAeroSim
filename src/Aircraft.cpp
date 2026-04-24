#include "Aircraft.hpp"
#include "liteaerosim.pb.h"
#include "navigation/WGS84.hpp"
#include "propulsion/PropulsionEDF.hpp"
#include "propulsion/PropulsionJet.hpp"
#include "propulsion/PropulsionProp.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

using liteaero::control::Mat21;

namespace liteaero::simulation {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Aircraft::Aircraft(std::unique_ptr<Propulsion> propulsion)
    : _propulsion(std::move(propulsion)) {}

// ---------------------------------------------------------------------------
// initialize()
// ---------------------------------------------------------------------------

void Aircraft::initialize(const nlohmann::json& config, float outer_dt_s) {
    _outer_dt_s = outer_dt_s;
    // 1. Inertia — read directly from config section (no schema_version in aircraft_config_v1)
    const auto& in_sec = config.at("inertia");
    _inertia.mass_kg   = in_sec.at("mass_kg").get<float>();
    _inertia.Ixx_kgm2  = in_sec.at("Ixx_kgm2").get<float>();
    _inertia.Iyy_kgm2  = in_sec.at("Iyy_kgm2").get<float>();
    _inertia.Izz_kgm2  = in_sec.at("Izz_kgm2").get<float>();

    // 2. Airframe performance — same pattern
    const auto& af_sec      = config.at("airframe");
    _airframe.g_max_nd      = af_sec.at("g_max_nd").get<float>();
    _airframe.g_min_nd      = af_sec.at("g_min_nd").get<float>();
    _airframe.tas_max_mps   = af_sec.at("tas_max_mps").get<float>();
    _airframe.mach_max_nd   = af_sec.at("mach_max_nd").get<float>();

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
    const float alpha_max_rad = lc.at("alpha_max_rad").get<float>();
    const float alpha_min_rad = lc.at("alpha_min_rad").get<float>();

    // 4. Aerodynamic performance and command-derivative filter config
    const auto& ac = config.at("aircraft");
    AeroPerformanceConfig aero_cfg;
    aero_cfg.s_ref_m2  = ac.at("S_ref_m2").get<float>();
    aero_cfg.ar        = ac.at("ar").get<float>();
    aero_cfg.e         = ac.at("e").get<float>();
    aero_cfg.cd0       = ac.at("cd0").get<float>();
    aero_cfg.cl_y_beta = ac.at("cl_y_beta").get<float>();
    const float S_ref_m2  = aero_cfg.s_ref_m2;
    const float cl_y_beta = aero_cfg.cl_y_beta;
    _aeroPerf.emplace(aero_cfg);

    _cmd_filter_substeps  = ac.at("cmd_filter_substeps").get<int>();
    if (_cmd_filter_substeps < 1)
        throw std::invalid_argument("Aircraft::initialize: cmd_filter_substeps must be >= 1");
    _nz_wn_rad_s         = ac.at("nz_wn_rad_s").get<float>();
    _nz_zeta_nd          = ac.at("nz_zeta_nd").get<float>();
    _ny_wn_rad_s         = ac.at("ny_wn_rad_s").get<float>();
    _ny_zeta_nd          = ac.at("ny_zeta_nd").get<float>();
    _roll_rate_wn_rad_s  = ac.at("roll_rate_wn_rad_s").get<float>();
    _roll_rate_zeta_nd   = ac.at("roll_rate_zeta_nd").get<float>();
    _cmd_filter_dt_s     = outer_dt_s / static_cast<float>(_cmd_filter_substeps);
    // Nyquist: wn * cmd_filter_dt_s must be < π for each axis.
    constexpr float kPi = 3.14159265f;
    if (_nz_wn_rad_s * _cmd_filter_dt_s >= kPi)
        throw std::invalid_argument(
            "Aircraft::initialize: nz_wn_rad_s * cmd_filter_dt_s must be < π (Nyquist)");
    if (_ny_wn_rad_s * _cmd_filter_dt_s >= kPi)
        throw std::invalid_argument(
            "Aircraft::initialize: ny_wn_rad_s * cmd_filter_dt_s must be < π (Nyquist)");
    if (_roll_rate_wn_rad_s * _cmd_filter_dt_s >= kPi)
        throw std::invalid_argument(
            "Aircraft::initialize: roll_rate_wn_rad_s * cmd_filter_dt_s must be < π (Nyquist)");
    _nz_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _nz_wn_rad_s, _nz_zeta_nd, 0.f);
    _ny_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _ny_wn_rad_s, _ny_zeta_nd, 0.f);
    _roll_rate_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _roll_rate_wn_rad_s, _roll_rate_zeta_nd, 0.f);
    _nz_filter.resetToInput(1.f);

    // 5. Load factor allocator (references _liftCurve — must be emplaced after step 3)
    const auto& lfa_sec = config.at("load_factor_allocator");
    const float alpha_dot_max_rad_s = lfa_sec.value("alpha_dot_max_rad_s", 0.0f);
    _allocator.emplace(*_liftCurve, S_ref_m2, cl_y_beta,
                       alpha_min_rad, alpha_max_rad,
                       alpha_dot_max_rad_s);

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

    // Optional Euler angles (ZYX: heading, pitch, roll) — all default to 0.
    const float heading_rad = is.value("heading_rad", 0.0f);
    const float pitch_rad   = is.value("pitch_rad",   0.0f);
    const float roll_rad    = is.value("roll_rad",    0.0f);
    const Eigen::Quaternionf q_nb =
        Eigen::AngleAxisf(heading_rad, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch_rad,   Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll_rad,    Eigen::Vector3f::UnitX());

    _state = KinematicState(
        0.0,
        datum,
        vel_NED,
        Eigen::Vector3f::Zero(),
        q_nb,
        Eigen::Vector3f::Zero()
    );
    _initial_state = _state;

    // 7. Landing gear (optional — only initialized when "landing_gear" section is present)
    if (config.contains("landing_gear")) {
        _landing_gear.initialize(config.at("landing_gear"));
        _has_landing_gear = true;
    } else {
        _has_landing_gear = false;
    }
}

// ---------------------------------------------------------------------------
// reset()
// ---------------------------------------------------------------------------

void Aircraft::reset() {
    _state = _initial_state;
    _nz_filter.resetToInput(1.f);
    _ny_filter.resetToInput(0.f);
    _roll_rate_filter.resetToInput(0.f);
    _allocator->reset();
    _propulsion->reset();
    if (_has_landing_gear) _landing_gear.reset();
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
    const float n_cmd   = std::clamp(cmd.n_z, _airframe.g_min_nd, _airframe.g_max_nd);
    const float n_y_cmd = std::clamp(cmd.n_y, _airframe.g_min_nd, _airframe.g_max_nd);

    // 4. Inner filter loop: 2nd-order LP command response for Nz, Ny, and roll rate.
    //    Runs _cmd_filter_substeps times at dt = outer_dt / substeps.
    float n_z_shaped       = 0.f;
    float n_y_shaped       = 0.f;
    float rollRate_filt_rps = 0.f;
    for (int i = 0; i < _cmd_filter_substeps; ++i) {
        n_z_shaped       = _nz_filter.step(n_cmd);
        n_y_shaped       = _ny_filter.step(n_y_cmd);
        rollRate_filt_rps = _roll_rate_filter.step(cmd.rollRate_Wind_rps);
    }
    // Analytical derivatives from filter state: ẏ[k] ≈ (H·(Φ·x[k]+Γ·u[k]) - y[k]) / dt_inner.
    // Uses only quantities already available after the substep loop — no additional lag.
    const float n_z_dot = (_nz_filter.h() * (_nz_filter.phi() * _nz_filter.x()
                            + _nz_filter.gamma() * n_cmd))(0, 0) / _cmd_filter_dt_s
                          - n_z_shaped / _cmd_filter_dt_s;
    const float n_y_dot = (_ny_filter.h() * (_ny_filter.phi() * _ny_filter.x()
                            + _ny_filter.gamma() * n_y_cmd))(0, 0) / _cmd_filter_dt_s
                          - n_y_shaped / _cmd_filter_dt_s;

    // 5. Solve for α and β
    LoadFactorInputs lfa_in;
    lfa_in.n_z      = n_z_shaped;
    lfa_in.n_y      = n_y_shaped;
    lfa_in.q_inf    = q_inf;
    lfa_in.thrust_n = _propulsion->thrust_n();   // previous-step thrust (0 on first call)
    lfa_in.mass_kg  = _inertia.mass_kg;
    lfa_in.n_z_dot  = n_z_dot;
    lfa_in.n_y_dot  = n_y_dot;
    lfa_in.dt_s     = _outer_dt_s;
    const LoadFactorOutputs lfa_out = _allocator->solve(lfa_in);

    // 6. Lift coefficient (effective CL accounts for stall and recovery)
    const float cl = lfa_out.cl_eff;

    // 7. Aerodynamic forces in Wind frame
    const AeroForces F =
        _aeroPerf->compute(lfa_out.alpha_rad, lfa_out.beta_rad, q_inf, cl);

    // 8. Landing gear contact forces (computed before propulsion so terrain query
    //    uses the pre-step kinematic state, consistent with quasi-static strut model).
    ContactForces contact_forces;
    if (_has_landing_gear && _terrain != nullptr) {
        contact_forces = _landing_gear.step(
            _state.snapshot(),
            *_terrain,
            0.0f,    // nose wheel steering angle (not yet wired to AircraftCommand)
            0.0f,    // brake left demand
            0.0f,    // brake right demand
            _outer_dt_s);
    }

    // 9. Advance propulsion
    const float T = _propulsion->step(cmd.throttle_nd, V_air, rho_kgm3);

    // 10. Wind-frame specific force = aero + thrust + gravity.
    //    Thrust decomposition (Wind frame, X forward, Y right, Z down):
    //      Tx =  T·cos(α)·cos(β)
    //      Ty = -T·cos(α)·sin(β)
    //      Tz = -T·sin(α)
    //    Gravity in Wind frame: C_WN · g_NED = R_nw_mat^T · {0, 0, g}
    //    (R_nw_mat is already computed below for the landing gear transform.)
    const float m  = _inertia.mass_kg;
    const float ca = std::cos(lfa_out.alpha_rad);
    const float sa = std::sin(lfa_out.alpha_rad);
    const float cb = std::cos(lfa_out.beta_rad);
    const float sb = std::sin(lfa_out.beta_rad);

    // Transform landing gear body-frame force to wind frame.
    // R_body_to_wind = R_nw^T * R_nb  (body→NED→wind)
    const Eigen::Matrix3f R_nb_mat = _state.q_nb().toRotationMatrix();
    const Eigen::Matrix3f R_nw_mat = _state.q_nw().toRotationMatrix();
    const Eigen::Vector3f F_gear_wind = R_nw_mat.transpose() * (R_nb_mat * contact_forces.force_body_n);

    constexpr float kGravity_mps2 = 9.80665f;
    const Eigen::Vector3f g_wind = R_nw_mat.transpose() * Eigen::Vector3f{0.f, 0.f, kGravity_mps2};

    const float ax = (T * ca * cb + F.x_n + F_gear_wind.x()) / m + g_wind.x();
    const float ay = (-T * ca * sb + F.y_n + F_gear_wind.y()) / m + g_wind.y();
    const float az = (-T * sa      + F.z_n + F_gear_wind.z()) / m + g_wind.z();

    // 11. Advance kinematic state
    _state.step(time_sec,
                Eigen::Vector3f{ax, ay, az},
                rollRate_filt_rps,
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
    j["cmd_filter_substeps"]  = _cmd_filter_substeps;
    j["cmd_filter_dt_s"]      = _cmd_filter_dt_s;
    j["nz_wn_rad_s"]          = _nz_wn_rad_s;
    j["nz_zeta_nd"]           = _nz_zeta_nd;
    j["ny_wn_rad_s"]          = _ny_wn_rad_s;
    j["ny_zeta_nd"]           = _ny_zeta_nd;
    j["roll_rate_wn_rad_s"]   = _roll_rate_wn_rad_s;
    j["roll_rate_zeta_nd"]    = _roll_rate_zeta_nd;
    j["nz_filter"]            = _nz_filter.serializeJson();
    j["ny_filter"]            = _ny_filter.serializeJson();
    j["roll_rate_filter"]     = _roll_rate_filter.serializeJson();
    j["airframe"]        = _airframe.serializeJson();
    j["inertia"]         = _inertia.serializeJson();
    if (_liftCurve)       j["lift_curve"]       = _liftCurve->serializeJson();
    if (_aeroPerf)        j["aero_performance"] = _aeroPerf->serializeJson();
    if (_allocator)       j["allocator"]         = _allocator->serializeJson();
    if (_propulsion)      j["propulsion"]         = _propulsion->serializeJson();
    if (_has_landing_gear) j["landing_gear_state"] = _landing_gear.serializeJson();
    return j;
}

void Aircraft::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1)
        throw std::runtime_error("Aircraft::deserializeJson: unsupported schema_version");
    if (j.at("type").get<std::string>() != "Aircraft")
        throw std::runtime_error("Aircraft::deserializeJson: unexpected type");

    _airframe = AirframePerformance::deserializeJson(j.at("airframe"));
    _inertia  = Inertia::deserializeJson(j.at("inertia"));

    _liftCurve.emplace(LiftCurveModel::deserializeJson(j.at("lift_curve")));
    _aeroPerf.emplace(AeroPerformance::deserializeJson(j.at("aero_performance")));

    // Emplace allocator with placeholder config — deserializeJson overwrites all scalar fields.
    // The reference to _liftCurve is set at construction and is the only field not restored.
    _allocator.emplace(*_liftCurve, 1.0f, -0.1f);
    _allocator->deserializeJson(j.at("allocator"));

    _state.deserializeJson(j.at("kinematic_state"));
    _initial_state.deserializeJson(j.at("initial_state"));
    _cmd_filter_substeps = j.at("cmd_filter_substeps").get<int>();
    _cmd_filter_dt_s     = j.at("cmd_filter_dt_s").get<float>();
    _nz_wn_rad_s         = j.at("nz_wn_rad_s").get<float>();
    _nz_zeta_nd          = j.at("nz_zeta_nd").get<float>();
    _ny_wn_rad_s         = j.at("ny_wn_rad_s").get<float>();
    _ny_zeta_nd          = j.at("ny_zeta_nd").get<float>();
    _roll_rate_wn_rad_s  = j.at("roll_rate_wn_rad_s").get<float>();
    _roll_rate_zeta_nd   = j.at("roll_rate_zeta_nd").get<float>();
    _nz_filter.deserializeJson(j.at("nz_filter"));
    _ny_filter.deserializeJson(j.at("ny_filter"));
    _roll_rate_filter.deserializeJson(j.at("roll_rate_filter"));

    if (_propulsion) {
        _propulsion->deserializeJson(j.at("propulsion"));
    }
    if (_has_landing_gear && j.contains("landing_gear_state")) {
        _landing_gear.deserializeJson(j.at("landing_gear_state"));
    }
}

// Helper: parse bytes into a proto sub-message of type T.
template <typename T>
static T parseSubMessage(const std::vector<uint8_t>& bytes) {
    T msg;
    msg.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()));
    return msg;
}

// Helper: serialize a proto sub-message to bytes.
template <typename T>
static std::vector<uint8_t> serializeSubMessage(const T& msg) {
    const std::string s = msg.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

std::vector<uint8_t> Aircraft::serializeProto() const {
    las_proto::AircraftState proto;
    proto.set_schema_version(1);
    proto.set_cmd_filter_substeps(_cmd_filter_substeps);
    proto.set_cmd_filter_dt_s(_cmd_filter_dt_s);
    proto.set_nz_wn_rad_s(_nz_wn_rad_s);
    proto.set_nz_zeta_nd(_nz_zeta_nd);
    proto.set_ny_wn_rad_s(_ny_wn_rad_s);
    proto.set_ny_zeta_nd(_ny_zeta_nd);
    proto.set_roll_rate_wn_rad_s(_roll_rate_wn_rad_s);
    proto.set_roll_rate_zeta_nd(_roll_rate_zeta_nd);
    proto.add_nz_filter_x(_nz_filter.x()(0, 0));
    proto.add_nz_filter_x(_nz_filter.x()(1, 0));
    proto.add_ny_filter_x(_ny_filter.x()(0, 0));
    proto.add_ny_filter_x(_ny_filter.x()(1, 0));
    proto.add_roll_rate_filter_x(_roll_rate_filter.x()(0, 0));
    proto.add_roll_rate_filter_x(_roll_rate_filter.x()(1, 0));

    *proto.mutable_kinematic_state()  = parseSubMessage<las_proto::KinematicState>(_state.serializeProto());
    *proto.mutable_initial_state()    = parseSubMessage<las_proto::KinematicState>(_initial_state.serializeProto());
    if (_allocator) *proto.mutable_allocator()         = parseSubMessage<las_proto::LoadFactorAllocatorState>(_allocator->serializeProto());
    if (_liftCurve) *proto.mutable_lift_curve()        = parseSubMessage<las_proto::LiftCurveParams>(_liftCurve->serializeProto());
    if (_aeroPerf)  *proto.mutable_aero_performance()  = parseSubMessage<las_proto::AeroPerformanceParams>(_aeroPerf->serializeProto());
    *proto.mutable_airframe() = parseSubMessage<las_proto::AirframePerformanceParams>(_airframe.serializeProto());
    *proto.mutable_inertia()  = parseSubMessage<las_proto::InertiaParams>(_inertia.serializeProto());

    if (_propulsion) {
        if (auto* jet = dynamic_cast<PropulsionJet*>(_propulsion.get()))
            *proto.mutable_jet()  = parseSubMessage<las_proto::PropulsionJetState>(jet->serializeProto());
        else if (auto* edf = dynamic_cast<PropulsionEDF*>(_propulsion.get()))
            *proto.mutable_edf()  = parseSubMessage<las_proto::PropulsionEdfState>(edf->serializeProto());
        else if (auto* prop = dynamic_cast<PropulsionProp*>(_propulsion.get()))
            *proto.mutable_prop() = parseSubMessage<las_proto::PropulsionPropState>(prop->serializeProto());
    }

    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

void Aircraft::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::AircraftState proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())))
        throw std::runtime_error("Aircraft::deserializeProto: failed to parse bytes");
    if (proto.schema_version() != 1)
        throw std::runtime_error("Aircraft::deserializeProto: unsupported schema_version");

    _airframe = AirframePerformance::deserializeProto(serializeSubMessage(proto.airframe()));
    _inertia  = Inertia::deserializeProto(serializeSubMessage(proto.inertia()));

    _liftCurve.emplace(LiftCurveModel::deserializeProto(serializeSubMessage(proto.lift_curve())));
    _aeroPerf.emplace(AeroPerformance::deserializeProto(serializeSubMessage(proto.aero_performance())));

    _allocator.emplace(*_liftCurve, 1.0f, -0.1f);
    _allocator->deserializeProto(serializeSubMessage(proto.allocator()));

    _state.deserializeProto(serializeSubMessage(proto.kinematic_state()));
    _initial_state.deserializeProto(serializeSubMessage(proto.initial_state()));
    _cmd_filter_substeps = proto.cmd_filter_substeps();
    _cmd_filter_dt_s     = proto.cmd_filter_dt_s();
    _nz_wn_rad_s         = proto.nz_wn_rad_s();
    _nz_zeta_nd          = proto.nz_zeta_nd();
    _ny_wn_rad_s         = proto.ny_wn_rad_s();
    _ny_zeta_nd          = proto.ny_zeta_nd();
    _roll_rate_wn_rad_s  = proto.roll_rate_wn_rad_s();
    _roll_rate_zeta_nd   = proto.roll_rate_zeta_nd();
    _nz_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _nz_wn_rad_s, _nz_zeta_nd, 0.f);
    _ny_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _ny_wn_rad_s, _ny_zeta_nd, 0.f);
    _roll_rate_filter.setLowPassSecondIIR(_cmd_filter_dt_s, _roll_rate_wn_rad_s, _roll_rate_zeta_nd, 0.f);
    Mat21 nz_x;
    nz_x(0, 0) = proto.nz_filter_x_size() >= 2 ? proto.nz_filter_x(0) : 0.f;
    nz_x(1, 0) = proto.nz_filter_x_size() >= 2 ? proto.nz_filter_x(1) : 0.f;
    _nz_filter.resetState(nz_x);
    Mat21 ny_x;
    ny_x(0, 0) = proto.ny_filter_x_size() >= 2 ? proto.ny_filter_x(0) : 0.f;
    ny_x(1, 0) = proto.ny_filter_x_size() >= 2 ? proto.ny_filter_x(1) : 0.f;
    _ny_filter.resetState(ny_x);
    Mat21 rr_x;
    rr_x(0, 0) = proto.roll_rate_filter_x_size() >= 2 ? proto.roll_rate_filter_x(0) : 0.f;
    rr_x(1, 0) = proto.roll_rate_filter_x_size() >= 2 ? proto.roll_rate_filter_x(1) : 0.f;
    _roll_rate_filter.resetState(rr_x);

    if (_propulsion) {
        switch (proto.propulsion_case()) {
            case las_proto::AircraftState::kJet:
                _propulsion->deserializeProto(serializeSubMessage(proto.jet()));
                break;
            case las_proto::AircraftState::kEdf:
                _propulsion->deserializeProto(serializeSubMessage(proto.edf()));
                break;
            case las_proto::AircraftState::kProp:
                _propulsion->deserializeProto(serializeSubMessage(proto.prop()));
                break;
            default:
                break;
        }
    }
}

} // namespace liteaero::simulation
