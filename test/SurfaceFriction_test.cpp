// Tests for SurfaceFriction and SurfaceFrictionUniform — Step B.
// Design authority: docs/architecture/landing_gear.md

#include "landing_gear/SurfaceFrictionUniform.hpp"
#include <gtest/gtest.h>

using namespace liteaero::simulation;

TEST(SurfaceFrictionUniform, ReturnsConfiguredCoefficients) {
    FrictionCoefficients expected{0.75f, 0.65f, 0.72f, 0.62f};
    SurfaceFrictionUniform model{expected};
    const auto got = model.frictionCoefficients({0.0f, 100.0f, -300.0f});
    EXPECT_FLOAT_EQ(got.longitudinal_peak_nd,    0.75f);
    EXPECT_FLOAT_EQ(got.longitudinal_sliding_nd, 0.65f);
    EXPECT_FLOAT_EQ(got.lateral_peak_nd,         0.72f);
    EXPECT_FLOAT_EQ(got.lateral_sliding_nd,      0.62f);
}

TEST(SurfaceFrictionUniform, CoefficientsIndependentOfPosition) {
    SurfaceFrictionUniform model = SurfaceFrictionUniform::pavement();
    const auto at_origin = model.frictionCoefficients(Eigen::Vector3f::Zero());
    const auto far_away  = model.frictionCoefficients({1000.0f, -2000.0f, 500.0f});
    EXPECT_FLOAT_EQ(at_origin.longitudinal_peak_nd, far_away.longitudinal_peak_nd);
}

TEST(SurfaceFrictionUniform, JsonRoundTrip) {
    const auto model    = SurfaceFrictionUniform::pavement(/*wet=*/false);
    const auto restored = SurfaceFrictionUniform::deserializeJson(model.serializeJson());

    const auto orig = model.frictionCoefficients(Eigen::Vector3f::Zero());
    const auto rest = restored.frictionCoefficients(Eigen::Vector3f::Zero());
    EXPECT_FLOAT_EQ(rest.longitudinal_peak_nd,    orig.longitudinal_peak_nd);
    EXPECT_FLOAT_EQ(rest.longitudinal_sliding_nd, orig.longitudinal_sliding_nd);
    EXPECT_FLOAT_EQ(rest.lateral_peak_nd,         orig.lateral_peak_nd);
    EXPECT_FLOAT_EQ(rest.lateral_sliding_nd,      orig.lateral_sliding_nd);
}

TEST(SurfaceFrictionUniform, WetPavementLessThanDryPavement) {
    const auto dry = SurfaceFrictionUniform::pavement(false).frictionCoefficients({});
    const auto wet = SurfaceFrictionUniform::pavement(true).frictionCoefficients({});
    EXPECT_LT(wet.longitudinal_peak_nd, dry.longitudinal_peak_nd);
    EXPECT_LT(wet.lateral_peak_nd,      dry.lateral_peak_nd);
}

TEST(SurfaceFrictionUniform, AllNamedConstructors_Positive) {
    for (bool wet : {false, true}) {
        EXPECT_GT(SurfaceFrictionUniform::pavement(wet).frictionCoefficients({}).longitudinal_peak_nd, 0.0f);
        EXPECT_GT(SurfaceFrictionUniform::grass(wet).frictionCoefficients({}).longitudinal_peak_nd,    0.0f);
        EXPECT_GT(SurfaceFrictionUniform::dirt(wet).frictionCoefficients({}).longitudinal_peak_nd,     0.0f);
        EXPECT_GT(SurfaceFrictionUniform::gravel(wet).frictionCoefficients({}).longitudinal_peak_nd,   0.0f);
    }
}
