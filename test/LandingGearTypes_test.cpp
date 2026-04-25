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
    p.attach_point_body_m   = {0.0f, 0.0f, 0.5f};
    p.travel_axis_body      = {0.0f, 0.0f, 1.0f};
    p.spring_stiffness_npm  = 10000.0f;
    p.damper_coeff_nspm     = 500.0f;
    p.preload_n             = 0.0f;
    p.travel_max_m          = 0.3f;
    p.tyre_radius_m         = 0.2f;
    p.rolling_resistance_nd = 0.02f;
    p.max_brake_torque_nm   = 0.0f;
    p.is_steerable          = false;
    p.has_brake             = false;
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
