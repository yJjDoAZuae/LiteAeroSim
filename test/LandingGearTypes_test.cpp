// Tests for ContactForces, StrutState, and WheelUnit data types — Step A.
// Design authority: docs/architecture/landing_gear.md

#include "physics/ContactForces.hpp"
#include "landing_gear/StrutState.hpp"
#include "landing_gear/WheelUnit.hpp"
#include <gtest/gtest.h>

using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// ContactForces
// ---------------------------------------------------------------------------

TEST(ContactForces, DefaultConstructed_AllZeroAndFalse) {
    ContactForces cf;
    EXPECT_EQ(cf.force_body_n,   Eigen::Vector3f::Zero());
    EXPECT_EQ(cf.moment_body_nm, Eigen::Vector3f::Zero());
    EXPECT_FALSE(cf.weight_on_wheels);
}

TEST(ContactForces, JsonRoundTrip) {
    ContactForces cf;
    cf.force_body_n      = {1.0f, 2.0f, -3.0f};
    cf.moment_body_nm    = {0.1f, 0.2f,  0.3f};
    cf.weight_on_wheels  = true;

    ContactForces restored;
    restored.deserializeJson(cf.serializeJson());

    EXPECT_FLOAT_EQ(restored.force_body_n.x(),    1.0f);
    EXPECT_FLOAT_EQ(restored.force_body_n.y(),    2.0f);
    EXPECT_FLOAT_EQ(restored.force_body_n.z(),   -3.0f);
    EXPECT_FLOAT_EQ(restored.moment_body_nm.x(),  0.1f);
    EXPECT_FLOAT_EQ(restored.moment_body_nm.y(),  0.2f);
    EXPECT_FLOAT_EQ(restored.moment_body_nm.z(),  0.3f);
    EXPECT_TRUE(restored.weight_on_wheels);
}

TEST(ContactForces, ProtoRoundTrip) {
    ContactForces cf;
    cf.force_body_n      = {-5.0f, 0.0f, 100.0f};
    cf.moment_body_nm    = {10.0f, -20.0f, 5.0f};
    cf.weight_on_wheels  = true;

    ContactForces restored;
    restored.deserializeProto(cf.serializeProto());

    EXPECT_FLOAT_EQ(restored.force_body_n.x(),    -5.0f);
    EXPECT_FLOAT_EQ(restored.force_body_n.z(),   100.0f);
    EXPECT_FLOAT_EQ(restored.moment_body_nm.y(), -20.0f);
    EXPECT_TRUE(restored.weight_on_wheels);
}

// ---------------------------------------------------------------------------
// StrutState
// ---------------------------------------------------------------------------

TEST(StrutState, DefaultConstructed_AllZero) {
    StrutState ss;
    EXPECT_FLOAT_EQ(ss.strut_deflection_m,        0.0f);
    EXPECT_FLOAT_EQ(ss.strut_deflection_rate_mps, 0.0f);
    EXPECT_FLOAT_EQ(ss.wheel_speed_rps,           0.0f);
}

TEST(StrutState, JsonRoundTrip) {
    StrutState ss{0.05f, 0.1f, 10.0f};
    StrutState restored;
    restored.deserializeJson(ss.serializeJson());
    EXPECT_FLOAT_EQ(restored.strut_deflection_m,        0.05f);
    EXPECT_FLOAT_EQ(restored.strut_deflection_rate_mps, 0.1f);
    EXPECT_FLOAT_EQ(restored.wheel_speed_rps,           10.0f);
}

TEST(StrutState, ProtoRoundTrip) {
    StrutState ss{0.12f, -0.3f, 25.0f};
    StrutState restored;
    restored.deserializeProto(ss.serializeProto());
    EXPECT_FLOAT_EQ(restored.strut_deflection_m,        0.12f);
    EXPECT_FLOAT_EQ(restored.strut_deflection_rate_mps, -0.3f);
    EXPECT_FLOAT_EQ(restored.wheel_speed_rps,           25.0f);
}

// ---------------------------------------------------------------------------
// WheelUnit
// ---------------------------------------------------------------------------

static WheelUnitParams makeWheelParams() {
    WheelUnitParams p;
    p.attach_point_body_m        = {0.0f, 0.0f, 0.5f};
    p.travel_axis_body           = {0.0f, 0.0f, 1.0f};
    p.spring_stiffness_npm       = 10000.0f;
    p.damping_compression_nspm            = 500.0f;
    p.damping_extension_nspm              = 100.0f;
    p.orifice_damping_compression_ns2pm2  = 0.0f;
    p.orifice_damping_extension_ns2pm2    = 0.0f;
    p.spring_nonlinearity_nd              = 0.0f;
    p.preload_n                  = 0.0f;
    p.travel_max_m               = 0.3f;
    p.tyre_radius_m              = 0.2f;
    p.rolling_resistance_nd      = 0.02f;
    p.max_brake_torque_nm        = 0.0f;
    p.is_steerable               = false;
    p.has_brake                  = false;
    return p;
}

TEST(WheelUnit, JsonRoundTrip_DefaultState) {
    WheelUnit wu;
    wu.initialize(makeWheelParams());
    auto j = wu.serializeJson();

    WheelUnit restored;
    restored.initialize(makeWheelParams());
    restored.deserializeJson(j);

    EXPECT_FLOAT_EQ(restored.strutState().strut_deflection_m, 0.0f);
    EXPECT_FLOAT_EQ(restored.strutState().wheel_speed_rps,    0.0f);
}

// Helper: upward surface normal in body frame (body-z positive = down, so up = -z)
static const Eigen::Vector3f kNormalUp{0.f, 0.f, -1.f};
static const Eigen::Vector3f kContactOrigin{0.f, 0.f, 0.5f};

TEST(WheelUnit, AsymmetricDamping_CompressionHigherThanExtension) {
    WheelUnitParams p = makeWheelParams();
    p.damping_compression_nspm = 1000.0f;
    p.damping_extension_nspm   = 100.0f;
    p.spring_nonlinearity_nd   = 0.0f;

    // --- Compression step: prev=0, pen=0.1 m, dt=0.02 s → delta_dot = +5 m/s ---
    WheelUnit wu_compress;
    wu_compress.initialize(p);
    const auto r_c = wu_compress.step(0.1f, kContactOrigin,
                                       Eigen::Vector3f::Zero(), kNormalUp,
                                       0.f, 0.f, 1.0f, 0.02f);
    const float F_compress = -r_c.force_body_n.z();

    // --- Extension step: preset prev=0.2 m, pen=0.1 m, dt=0.02 s → delta_dot = -5 m/s ---
    WheelUnit wu_extend;
    wu_extend.initialize(p);
    wu_extend.setStrutState({0.2f, 0.0f, 0.0f});
    const auto r_e = wu_extend.step(0.1f, kContactOrigin,
                                     Eigen::Vector3f::Zero(), kNormalUp,
                                     0.f, 0.f, 1.0f, 0.02f);
    const float F_extend = -r_e.force_body_n.z();

    EXPECT_GT(F_compress, F_extend)
        << "Compression force must exceed extension force at equal |delta_dot|";
}

TEST(WheelUnit, OrificesDamping_ForceScalesQuadratically) {
    // Orifice force = c * |delta_dot| * delta_dot.
    // At 2× velocity, force must be 4× higher (quadratic).
    WheelUnitParams p = makeWheelParams();
    p.damping_compression_nspm          = 0.0f;
    p.damping_extension_nspm            = 0.0f;
    p.orifice_damping_compression_ns2pm2 = 500.0f;
    p.orifice_damping_extension_ns2pm2   = 500.0f;
    p.spring_nonlinearity_nd            = 0.0f;

    // Step at 1 m/s compression: prev=0, pen = 1*dt, dt=0.01s → delta_dot = 1 m/s
    WheelUnit wu1;
    wu1.initialize(p);
    const auto r1 = wu1.step(0.01f, kContactOrigin, Eigen::Vector3f::Zero(),
                              kNormalUp, 0.f, 0.f, 1.0f, 0.01f);
    const float F1 = -r1.force_body_n.z();

    // Step at 2 m/s compression: prev=0, pen = 2*dt → delta_dot = 2 m/s
    WheelUnit wu2;
    wu2.initialize(p);
    const auto r2 = wu2.step(0.02f, kContactOrigin, Eigen::Vector3f::Zero(),
                              kNormalUp, 0.f, 0.f, 1.0f, 0.01f);
    const float F2 = -r2.force_body_n.z();

    // F_orifice = c * v^2.  With spring contribution also present, verify F2 > 3*F1
    // (spring contribution grows linearly while orifice grows quadratically)
    EXPECT_GT(F2, 3.0f * F1)
        << "Orifice damping force must grow faster than linearly with velocity";
}

TEST(WheelUnit, NonlinearSpring_HardensAtFullCompression) {
    const Eigen::Vector3f contact{0.f, 0.f, 0.5f};

    WheelUnitParams p_lin = makeWheelParams();
    p_lin.damping_compression_nspm = 0.0f;
    p_lin.damping_extension_nspm   = 0.0f;
    p_lin.spring_nonlinearity_nd   = 0.0f;

    WheelUnitParams p_nl = p_lin;
    p_nl.spring_nonlinearity_nd    = 2.0f;

    WheelUnit wu_lin, wu_nl;
    wu_lin.initialize(p_lin);
    wu_nl.initialize(p_nl);

    // At full compression (delta = travel_max = 0.3 m):
    //   F_lin = k * 0.3               = 3000 N
    //   F_nl  = k * 0.3 * (1 + 2*1²) = 9000 N
    const auto r_lin = wu_lin.step(p_lin.travel_max_m, contact,
                                    Eigen::Vector3f::Zero(), kNormalUp,
                                    0.f, 0.f, 1.0f, 0.02f);
    const auto r_nl  = wu_nl.step(p_nl.travel_max_m, contact,
                                   Eigen::Vector3f::Zero(), kNormalUp,
                                   0.f, 0.f, 1.0f, 0.02f);

    EXPECT_GT(-r_nl.force_body_n.z(), -r_lin.force_body_n.z())
        << "Nonlinear spring must produce higher force at full compression";
}

TEST(WheelUnit, ProtoRoundTrip_NonZeroState) {
    WheelUnit wu;
    wu.initialize(makeWheelParams());
    wu.setStrutState({0.08f, 0.5f, 15.0f});

    WheelUnit restored;
    restored.initialize(makeWheelParams());
    restored.deserializeProto(wu.serializeProto());

    EXPECT_FLOAT_EQ(restored.strutState().strut_deflection_m,        0.08f);
    EXPECT_FLOAT_EQ(restored.strutState().strut_deflection_rate_mps, 0.5f);
    EXPECT_FLOAT_EQ(restored.strutState().wheel_speed_rps,           15.0f);
}
