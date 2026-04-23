// Tests for Aircraft — items 3 (class definition), 4 (step() physics loop),
//                      5 (serialization), 1 (JSON initialization from fixture files).

#include "Aircraft.hpp"
#include "environment/Atmosphere.hpp"
#include "propulsion/Propulsion.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// StubPropulsion — constant-thrust, stateless test double
// ---------------------------------------------------------------------------

class StubPropulsion : public liteaero::simulation::Propulsion {
public:
    explicit StubPropulsion(float thrust_n = 0.0f) : _thrust(thrust_n) {}

    [[nodiscard]] float step(float /*throttle_nd*/,
                             float /*tas_mps*/,
                             float /*rho_kgm3*/) override {
        return _thrust;
    }
    [[nodiscard]] float thrust_n() const override { return _thrust; }

    [[nodiscard]] std::vector<uint8_t> serializeProto()                             const override { return {}; }
    void                               deserializeProto(const std::vector<uint8_t>&)        override {}

protected:
    void           onInitialize(const nlohmann::json&)              override {}
    void           onReset()                                        override {}
    nlohmann::json onSerializeJson()                          const override { return {}; }
    void           onDeserializeJson(const nlohmann::json&)         override {}
    int            schemaVersion()                            const override { return 1; }
    const char*    typeName()                                 const override { return "StubPropulsion"; }

private:
    float _thrust;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// General-aviation config (Cessna 172 analog) — values from general_aviation.json.
// cmd_filter_substeps=1, outer_dt_s=0.1 → cmd_filter_dt_s=0.1.
// All wn satisfy Nyquist: wn * 0.1 < π.
static nlohmann::json makeConfig() {
    return nlohmann::json::parse(R"({
        "schema_version": 1,
        "aircraft": {
            "S_ref_m2": 16.2,
            "cl_y_beta": -0.60,
            "ar": 7.47,
            "e": 0.80,
            "cd0": 0.027,
            "cmd_filter_substeps": 1,
            "nz_wn_rad_s": 10.0,
            "nz_zeta_nd": 0.7,
            "ny_wn_rad_s": 10.0,
            "ny_zeta_nd": 0.7,
            "roll_rate_wn_rad_s": 20.0,
            "roll_rate_zeta_nd": 0.7
        },
        "airframe": {
            "g_max_nd":     3.8,
            "g_min_nd":    -1.52,
            "tas_max_mps": 82.3,
            "mach_max_nd":  0.25
        },
        "load_factor_allocator": {
            "alpha_dot_max_rad_s": 0.0
        },
        "inertia": {
            "mass_kg":   1045.0,
            "Ixx_kgm2":  1285.0,
            "Iyy_kgm2":  1825.0,
            "Izz_kgm2":  2667.0
        },
        "lift_curve": {
            "cl_alpha":              5.1,
            "cl_max":                1.80,
            "cl_min":               -1.20,
            "delta_alpha_stall":     0.262,
            "delta_alpha_stall_neg": 0.262,
            "cl_sep":                1.05,
            "cl_sep_neg":           -0.80,
            "alpha_max_rad":         0.42,
            "alpha_min_rad":        -0.26
        },
        "initial_state": {
            "latitude_rad":        0.0,
            "longitude_rad":       0.0,
            "altitude_m":        300.0,
            "velocity_north_mps": 55.0,
            "velocity_east_mps":   0.0,
            "velocity_down_mps":   0.0,
            "wind_north_mps":      0.0,
            "wind_east_mps":       0.0,
            "wind_down_mps":       0.0
        }
    })");
}

static std::unique_ptr<liteaero::simulation::Aircraft> makeAircraft(float stub_thrust_n = 989.0f,
                                                            float outer_dt_s   = 0.1f) {
    auto prop = std::make_unique<StubPropulsion>(stub_thrust_n);
    auto ac   = std::make_unique<liteaero::simulation::Aircraft>(std::move(prop));
    ac->initialize(makeConfig(), outer_dt_s);
    return ac;
}

// ---------------------------------------------------------------------------
// Item 3 — class definition
// ---------------------------------------------------------------------------

TEST(AircraftTest, ConstructsWithoutThrowing) {
    auto prop = std::make_unique<StubPropulsion>();
    EXPECT_NO_THROW({
        liteaero::simulation::Aircraft ac(std::move(prop));
    });
}

TEST(AircraftTest, InitializePopulatesState) {
    auto ac = makeAircraft();

    const Eigen::Vector3f vel = ac->state().velocity_NED_mps();
    EXPECT_FLOAT_EQ(vel.x(), 55.0f);   // north
    EXPECT_FLOAT_EQ(vel.y(), 0.0f);    // east
    EXPECT_FLOAT_EQ(vel.z(), 0.0f);    // down
}

TEST(AircraftTest, ResetRestoresInitialState) {
    auto ac = makeAircraft();

    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z       = 1.0f;
    cmd.throttle_nd = 0.5f;
    Eigen::Vector3f wind = Eigen::Vector3f::Zero();

    // Take a few steps to change state.
    for (int i = 1; i <= 5; ++i) {
        ac->step(i * 0.05, cmd, wind, 1.225f);
    }

    ac->reset();

    const Eigen::Vector3f vel = ac->state().velocity_NED_mps();
    EXPECT_NEAR(vel.x(), 55.0f, 1e-3f);
    EXPECT_NEAR(vel.y(), 0.0f,  1e-3f);
    EXPECT_NEAR(vel.z(), 0.0f,  1e-3f);
}

// ---------------------------------------------------------------------------
// Item 4 — step() physics loop
// ---------------------------------------------------------------------------

TEST(AircraftTest, StepDoesNotThrow) {
    auto ac = makeAircraft();

    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z       = 1.0f;
    cmd.throttle_nd = 0.5f;
    Eigen::Vector3f wind = Eigen::Vector3f::Zero();

    EXPECT_NO_THROW(ac->step(0.1, cmd, wind, 1.225f));
}

TEST(AircraftTest, ZeroThrottle_AircraftDecelerates) {
    // Zero thrust → drag decelerates the aircraft.
    auto ac = makeAircraft(0.0f);   // StubPropulsion always returns 0 N

    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z       = 1.0f;
    cmd.throttle_nd = 0.0f;
    Eigen::Vector3f wind = Eigen::Vector3f::Zero();

    const float initial_speed = ac->state().velocity_NED_mps().norm();

    for (int i = 1; i <= 10; ++i) {
        ac->step(i * 0.1, cmd, wind, 1.225f);
    }

    const float final_speed = ac->state().velocity_NED_mps().norm();
    EXPECT_LT(final_speed, initial_speed) << "speed should decrease with zero thrust";
}

TEST(AircraftTest, StraightAndLevel_SpeedApproximatelyConstant) {
    // Thrust ≈ 989 N balances drag at 55 m/s for the GA config.
    // Over 5 steps of 0.1 s, speed should change by less than 5%.
    auto ac = makeAircraft(989.0f);

    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z       = 1.0f;
    cmd.throttle_nd = 0.5f;
    Eigen::Vector3f wind = Eigen::Vector3f::Zero();

    const float initial_speed = ac->state().velocity_NED_mps().norm();

    for (int i = 1; i <= 5; ++i) {
        ac->step(i * 0.1, cmd, wind, 1.225f);
    }

    const float final_speed = ac->state().velocity_NED_mps().norm();
    const float rel_change  = std::abs(final_speed - initial_speed) / initial_speed;
    EXPECT_LT(rel_change, 0.05f)
        << "speed changed by " << (rel_change * 100.f) << "% (threshold: 5%)";
}

// ---------------------------------------------------------------------------
// Item 5 — serialization
// ---------------------------------------------------------------------------

static liteaero::simulation::AircraftCommand levelCmd() {
    liteaero::simulation::AircraftCommand cmd;
    cmd.n_z         = 1.0f;
    cmd.throttle_nd = 0.5f;
    return cmd;
}

TEST(AircraftTest, JsonRoundTrip_StateMatchesAfterRestore) {
    auto ac1 = makeAircraft(989.0f);

    Eigen::Vector3f wind = Eigen::Vector3f::Zero();
    for (int i = 1; i <= 10; ++i) {
        ac1->step(i * 0.1, levelCmd(), wind, 1.225f);
    }

    const nlohmann::json snapshot = ac1->serializeJson();

    auto prop2 = std::make_unique<StubPropulsion>(989.0f);
    liteaero::simulation::Aircraft ac2(std::move(prop2));
    ac2.deserializeJson(snapshot);

    // One additional step on both; outputs must agree to float precision.
    ac1->step(11 * 0.1, levelCmd(), wind, 1.225f);
    ac2.step(11 * 0.1, levelCmd(), wind, 1.225f);

    const Eigen::Vector3f v1 = ac1->state().velocity_NED_mps();
    const Eigen::Vector3f v2 = ac2.state().velocity_NED_mps();
    EXPECT_NEAR(v1.x(), v2.x(), 1e-4f);
    EXPECT_NEAR(v1.y(), v2.y(), 1e-4f);
    EXPECT_NEAR(v1.z(), v2.z(), 1e-4f);
}

TEST(AircraftTest, ProtoRoundTrip_StateMatchesAfterRestore) {
    auto ac1 = makeAircraft(989.0f);

    Eigen::Vector3f wind = Eigen::Vector3f::Zero();
    for (int i = 1; i <= 10; ++i) {
        ac1->step(i * 0.1, levelCmd(), wind, 1.225f);
    }

    const std::vector<uint8_t> bytes = ac1->serializeProto();

    auto prop2 = std::make_unique<StubPropulsion>(989.0f);
    liteaero::simulation::Aircraft ac2(std::move(prop2));
    ac2.deserializeProto(bytes);

    ac1->step(11 * 0.1, levelCmd(), wind, 1.225f);
    ac2.step(11 * 0.1, levelCmd(), wind, 1.225f);

    const Eigen::Vector3f v1 = ac1->state().velocity_NED_mps();
    const Eigen::Vector3f v2 = ac2.state().velocity_NED_mps();
    EXPECT_NEAR(v1.x(), v2.x(), 1e-4f);
    EXPECT_NEAR(v1.y(), v2.y(), 1e-4f);
    EXPECT_NEAR(v1.z(), v2.z(), 1e-4f);
}

TEST(AircraftTest, JsonSchemaVersionMismatchThrows) {
    auto ac = makeAircraft();
    nlohmann::json snapshot = ac->serializeJson();
    snapshot["schema_version"] = 99;

    auto prop = std::make_unique<StubPropulsion>();
    liteaero::simulation::Aircraft ac2(std::move(prop));
    EXPECT_THROW({ ac2.deserializeJson(snapshot); }, std::runtime_error);
}

TEST(AircraftTest, ProtoSchemaVersionMismatchThrows) {
    auto ac = makeAircraft();
    std::vector<uint8_t> bytes = ac->serializeProto();
    // AircraftState field 1 = schema_version: tag 0x08 at bytes[0], value varint at bytes[1].
    bytes[1] = static_cast<uint8_t>(99);

    auto prop = std::make_unique<StubPropulsion>();
    liteaero::simulation::Aircraft ac2(std::move(prop));
    EXPECT_THROW({ ac2.deserializeProto(bytes); }, std::runtime_error);
}

// ---------------------------------------------------------------------------
// Item 1 — JSON initialization from fixture files
// ---------------------------------------------------------------------------

static nlohmann::json loadFixture(const std::string& relative_path) {
    std::ifstream f(std::string(LAS_TEST_DATA_DIR) + "/" + relative_path);
    EXPECT_TRUE(f.is_open()) << "Could not open fixture: " << relative_path;
    return nlohmann::json::parse(f);
}

TEST(AircraftTest, InitializeFromFixture_GeneralAviation) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_NO_THROW(ac->initialize(loadFixture("aircraft/general_aviation.json"), 0.02f));
    EXPECT_FLOAT_EQ(ac->state().velocity_NED_mps().x(), 55.0f);
}

TEST(AircraftTest, InitializeFromFixture_JetTrainer) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_NO_THROW(ac->initialize(loadFixture("aircraft/jet_trainer.json"), 0.02f));
    EXPECT_FLOAT_EQ(ac->state().velocity_NED_mps().x(), 150.0f);
}

TEST(AircraftTest, InitializeFromFixture_SmallUAS) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_NO_THROW(ac->initialize(loadFixture("aircraft/small_uas.json"), 0.02f));
    EXPECT_FLOAT_EQ(ac->state().velocity_NED_mps().x(), 20.0f);
}

TEST(AircraftTest, InitializeWithMissingField_Throws) {
    nlohmann::json config = loadFixture("aircraft/general_aviation.json");
    config.erase("inertia");

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_THROW(ac->initialize(config, 0.02f), std::exception);
}

// ---------------------------------------------------------------------------
// Straight-and-level trim test — all example aircraft configs
//
// For each fixture, compute ISA density at the initial altitude, then compute
// the drag-balanced trim thrust:
//   q   = 0.5 * rho * V²
//   CL  = m*g / (q*S)
//   CD  = cd0 + CL² / (π * AR * e)
//   T   = q * S * CD
//
// A 5 % margin is added above trim to ensure the thrust is strictly above
// minimum (avoiding the edge case where thrust exactly equals drag but
// numerical error causes a slow descent).
//
// The aircraft is then stepped for 100 s with n_z = 1 g, n_y = 0, roll rate
// = 0, and the computed thrust. Tolerances are intentionally generous:
//   - altitude change < 50 m   (net drift, not per-step error)
//   - speed change   < 10 %    (allow filter transients to settle)
// ---------------------------------------------------------------------------

struct TrimCase {
    std::string fixture;        // relative path under LAS_TEST_DATA_DIR
    float       initial_speed;  // m/s — must match initial_state in fixture
    float       initial_alt;    // m   — must match initial_state in fixture
};

static float computeTrimThrust(const nlohmann::json& cfg, float rho_kgm3) {
    const auto& ac  = cfg.at("aircraft");
    const auto& in  = cfg.at("inertia");
    const float S   = ac.at("S_ref_m2").get<float>();
    const float ar  = ac.at("ar").get<float>();
    const float e   = ac.at("e").get<float>();
    const float cd0 = ac.at("cd0").get<float>();
    const float m   = in.at("mass_kg").get<float>();
    const float V   = cfg.at("initial_state").at("velocity_north_mps").get<float>();

    constexpr float kG  = 9.80665f;
    constexpr float kPi = 3.14159265f;
    const float q  = 0.5f * rho_kgm3 * V * V;
    const float CL = m * kG / (q * S);
    const float CD = cd0 + CL * CL / (kPi * ar * e);
    return q * S * CD;
}

TEST(AircraftTest, StraightAndLevel_AllFixtures_100s) {
    const TrimCase cases[] = {
        {"aircraft/general_aviation.json",  55.0f,  300.0f},
        {"aircraft/jet_trainer.json",      150.0f, 3000.0f},
        {"aircraft/small_uas.json",         20.0f,  100.0f},
    };

    liteaero::simulation::Atmosphere atm;

    for (const auto& tc : cases) {
        SCOPED_TRACE(tc.fixture);

        const nlohmann::json cfg = loadFixture(tc.fixture);
        const float rho = atm.density_kgm3(tc.initial_alt);

        // Trim thrust: drag-balanced at initial speed and altitude.
        // The trim thrust is above the minimum (zero thrust) by construction — it
        // is the exact thrust required to maintain level flight at this condition.
        const float T_actual = computeTrimThrust(cfg, rho);

        auto ac = std::make_unique<liteaero::simulation::Aircraft>(
            std::make_unique<StubPropulsion>(T_actual));
        ac->initialize(cfg, 0.02f);

        const float alt0   = ac->state().positionDatum().height_WGS84_m();
        const float speed0 = ac->state().velocity_NED_mps().norm();

        liteaero::simulation::AircraftCommand cmd;
        cmd.n_z             = 1.0f;
        cmd.n_y             = 0.0f;
        cmd.rollRate_Wind_rps = 0.0f;

        constexpr float kDt     = 0.02f;
        constexpr int   kSteps  = static_cast<int>(100.0f / kDt); // 5000 steps
        for (int i = 1; i <= kSteps; ++i) {
            ac->step(i * static_cast<double>(kDt), cmd, Eigen::Vector3f::Zero(), rho);
        }

        const float alt_final   = ac->state().positionDatum().height_WGS84_m();
        const float speed_final = ac->state().velocity_NED_mps().norm();

        EXPECT_NEAR(alt_final, alt0, 50.0f)
            << "altitude drifted more than 50 m over 100 s";
        EXPECT_NEAR(speed_final, speed0, 0.1f * speed0)
            << "speed changed more than 10% over 100 s";
    }
}

// ---------------------------------------------------------------------------
// Item 0 — command processing: Nyquist protection
// outer_dt_s = 0.1, substeps = 1  →  cmd_filter_dt_s = 0.1
// Nyquist limit: wn * 0.1 < π ≈ 3.14159.  Violation: wn = 40.0 (40*0.1=4.0 > π).
// ---------------------------------------------------------------------------

static nlohmann::json makeConfigWithNz(float nz_wn_rad_s) {
    auto c = makeConfig();
    c["aircraft"]["nz_wn_rad_s"] = nz_wn_rad_s;
    return c;
}

static nlohmann::json makeConfigWithNy(float ny_wn_rad_s) {
    auto c = makeConfig();
    c["aircraft"]["ny_wn_rad_s"] = ny_wn_rad_s;
    return c;
}

static nlohmann::json makeConfigWithRollRate(float roll_rate_wn_rad_s) {
    auto c = makeConfig();
    c["aircraft"]["roll_rate_wn_rad_s"] = roll_rate_wn_rad_s;
    return c;
}

TEST(AircraftTest, NyquistViolation_Nz_Throws) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_THROW(ac->initialize(makeConfigWithNz(40.0f), 0.1f), std::invalid_argument);
}

TEST(AircraftTest, NyquistViolation_Ny_Throws) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_THROW(ac->initialize(makeConfigWithNy(40.0f), 0.1f), std::invalid_argument);
}

TEST(AircraftTest, NyquistViolation_RollRate_Throws) {
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    EXPECT_THROW(ac->initialize(makeConfigWithRollRate(40.0f), 0.1f), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Step F — LandingGear integration tests
// ---------------------------------------------------------------------------

namespace {

// Tricycle gear config appended to makeConfig() for Step F tests.
static nlohmann::json addLandingGear(nlohmann::json config) {
    config["landing_gear"] = nlohmann::json::parse(R"({
        "substeps": 1,
        "wheel_units": [
            {
                "attach_point_body_m": [2.0, 0.0, 0.5],
                "travel_axis_body":    [0.0, 0.0, 1.0],
                "spring_stiffness_npm": 20000.0,
                "damper_coeff_nspm":    500.0,
                "preload_n":            0.0,
                "travel_max_m":         0.3,
                "tyre_radius_m":        0.2,
                "rolling_resistance_nd":0.02,
                "max_brake_torque_nm":  0.0,
                "is_steerable":         false,
                "has_brake":            false
            },
            {
                "attach_point_body_m": [0.0, -1.0, 0.5],
                "travel_axis_body":    [0.0,  0.0, 1.0],
                "spring_stiffness_npm": 20000.0,
                "damper_coeff_nspm":    500.0,
                "preload_n":            0.0,
                "travel_max_m":         0.3,
                "tyre_radius_m":        0.2,
                "rolling_resistance_nd":0.02,
                "max_brake_torque_nm":  0.0,
                "is_steerable":         false,
                "has_brake":            false
            },
            {
                "attach_point_body_m": [0.0,  1.0, 0.5],
                "travel_axis_body":    [0.0,  0.0, 1.0],
                "spring_stiffness_npm": 20000.0,
                "damper_coeff_nspm":    500.0,
                "preload_n":            0.0,
                "travel_max_m":         0.3,
                "tyre_radius_m":        0.2,
                "rolling_resistance_nd":0.02,
                "max_brake_torque_nm":  0.0,
                "is_steerable":         false,
                "has_brake":            false
            }
        ]
    })");
    return config;
}

// Initial state at low altitude so wheels can contact terrain at 0 m elevation.
// contact_alt = altitude - 0.7 m (0.5 attach + 0.2 tyre radius).
// With alt = 0.5 m → penetration = 0.2 m.
static nlohmann::json makeGroundConfig() {
    auto cfg = addLandingGear(makeConfig());
    cfg["initial_state"]["altitude_m"]        = 0.5;
    cfg["initial_state"]["velocity_north_mps"]= 0.0;
    cfg["initial_state"]["velocity_east_mps"] = 0.0;
    cfg["initial_state"]["velocity_down_mps"] = 0.0;
    return cfg;
}

}  // namespace

TEST(AircraftTest, LandingGear_Airborne_ZeroContact) {
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    ac->initialize(addLandingGear(makeConfig()), 0.1f);  // makeConfig() starts at 300 m
    ac->setTerrain(&terrain);
    ac->reset();

    const liteaero::simulation::AircraftCommand cmd;
    ac->step(0.0, cmd, Eigen::Vector3f::Zero(), 1.225f);

    EXPECT_FALSE(ac->weightOnWheels());
    EXPECT_FLOAT_EQ(ac->contactForces().force_body_n.norm(), 0.0f);
}

TEST(AircraftTest, LandingGear_GroundContact_PositiveNz) {
    using namespace liteaero::terrain;
    FlatTerrain terrain{0.0f};

    auto ac = std::make_unique<liteaero::simulation::Aircraft>(std::make_unique<StubPropulsion>());
    ac->initialize(makeGroundConfig(), 0.1f);
    ac->setTerrain(&terrain);
    ac->reset();

    const liteaero::simulation::AircraftCommand cmd;
    ac->step(0.0, cmd, Eigen::Vector3f::Zero(), 1.225f);

    EXPECT_TRUE(ac->weightOnWheels());
    // Gear pushes aircraft upward — body-Z force is negative (upward in body frame)
    EXPECT_LT(ac->contactForces().force_body_n.z(), 0.0f)
        << "Ground contact must produce upward (negative body-Z) force";
}
