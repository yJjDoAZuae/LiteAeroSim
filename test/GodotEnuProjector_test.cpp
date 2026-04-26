// GodotEnuProjector_test.cpp — Tests for liteaero::projection::GodotEnuProjector.
//
// Verifies the full ECEF -> ENU projection at the configured world origin and
// the glTF axis permutation (X=East, Y=Up, Z=-North), including the curvature
// drop d^2/(2R) that drives the LS-Issue 7 fix.

#define _USE_MATH_DEFINES
#include "projection/GodotEnuProjector.hpp"

#include "geodesy/Wgs84.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace liteaero::projection;
using liteaero::geodesy::kWgs84A;

// ---------------------------------------------------------------------------
// Identity at origin
// ---------------------------------------------------------------------------

TEST(GodotEnuProjectorTest, ProjectAtOrigin_ReturnsZero) {
    const ViewerOrigin origin{0.0, 0.0, 0.0};
    GodotEnuProjector projector(origin);

    const auto p = projector.project(0.0, 0.0, 0.0f);
    EXPECT_NEAR(p.x_m, 0.0f, 1e-3f);
    EXPECT_NEAR(p.y_m, 0.0f, 1e-3f);
    EXPECT_NEAR(p.z_m, 0.0f, 1e-3f);
}

TEST(GodotEnuProjectorTest, ProjectAltitudeAboveOrigin_PutsAltitudeOnY) {
    const ViewerOrigin origin{0.0, 0.0, 0.0};
    GodotEnuProjector projector(origin);

    const auto p = projector.project(0.0, 0.0, 1000.0f);
    EXPECT_NEAR(p.x_m,    0.0f, 1e-2f);
    EXPECT_NEAR(p.y_m, 1000.0f, 1e-2f);
    EXPECT_NEAR(p.z_m,    0.0f, 1e-2f);
}

// ---------------------------------------------------------------------------
// Axis convention: glTF / Godot X=East, Y=Up, Z=-North
// ---------------------------------------------------------------------------

TEST(GodotEnuProjectorTest, ProjectEastOfOrigin_PutsOffsetOnPositiveX) {
    const ViewerOrigin origin{0.0, 0.0, 0.0};
    GodotEnuProjector projector(origin);

    const double east_m = 1000.0;
    const double dlon   = east_m / kWgs84A;  // 1 km east at the equator

    const auto p = projector.project(0.0, dlon, 0.0f);
    EXPECT_NEAR(p.x_m, static_cast<float>(east_m), 1.0f);   // east -> +X
    // Curvature drop ~78 mm at 1 km; well within tolerance.
    EXPECT_NEAR(p.y_m,    0.0f, 1.0f);
    EXPECT_NEAR(p.z_m,    0.0f, 0.1f);
}

TEST(GodotEnuProjectorTest, ProjectNorthOfOrigin_PutsOffsetOnNegativeZ) {
    const ViewerOrigin origin{0.0, 0.0, 0.0};
    GodotEnuProjector projector(origin);

    // North distance at the equator uses the meridional radius
    // M = a * (1 - e^2) / (1 - e^2 sin^2 phi)^(3/2); at phi=0 this is
    // a * (1 - e^2).  Using this gives exactly 1000 m of north distance.
    const double north_m = 1000.0;
    const double M_eq    = kWgs84A * (1.0 - liteaero::geodesy::kWgs84E2);
    const double dlat    = north_m / M_eq;

    const auto p = projector.project(dlat, 0.0, 0.0f);
    EXPECT_NEAR(p.x_m,    0.0f, 0.1f);
    EXPECT_NEAR(p.y_m,    0.0f, 1.0f);
    EXPECT_NEAR(p.z_m, -static_cast<float>(north_m), 1.0f);  // north -> -Z
}

// ---------------------------------------------------------------------------
// Curvature drop — the geometry that drives Issue 7
// ---------------------------------------------------------------------------

TEST(GodotEnuProjectorTest, ProjectFarEastAtEllipsoid_ShowsCurvatureDrop) {
    const ViewerOrigin origin{0.0, 0.0, 0.0};
    GodotEnuProjector projector(origin);

    const double d    = 25000.0;
    const double dlon = d / kWgs84A;

    // Point on the ellipsoid 25 km east at h=0.  In viewer coordinates the
    // Y component (up) must be -d^2/(2R) ~= -49 m, NOT zero.  This is the
    // entire point of LS-T4: the projector is curvature-aware so this drop
    // matches the terrain GLB tile placement.
    const auto p = projector.project(0.0, dlon, 0.0f);

    const float expected_drop = static_cast<float>(d * d / (2.0 * kWgs84A));
    EXPECT_NEAR(p.x_m, static_cast<float>(d),  1.0f);
    EXPECT_NEAR(p.y_m, -expected_drop,         1.0f);
    EXPECT_NEAR(p.z_m, 0.0f,                   1e-3f);
}

// ---------------------------------------------------------------------------
// Mid-latitude origin (KSBA-ish)
// ---------------------------------------------------------------------------

TEST(GodotEnuProjectorTest, ProjectAtNonZeroOrigin_AtOriginIsZero) {
    const ViewerOrigin origin{
        34.4258 * M_PI / 180.0,
       -119.84  * M_PI / 180.0,
        100.0
    };
    GodotEnuProjector projector(origin);

    const auto p = projector.project(origin.lat_rad, origin.lon_rad,
                                     static_cast<float>(origin.height_wgs84_m));
    EXPECT_NEAR(p.x_m, 0.0f, 1e-2f);
    EXPECT_NEAR(p.y_m, 0.0f, 1e-2f);
    EXPECT_NEAR(p.z_m, 0.0f, 1e-2f);
}

TEST(GodotEnuProjectorTest, ProjectAtNonZeroOrigin_AltitudeAboveYieldsYUp) {
    const ViewerOrigin origin{
        34.4258 * M_PI / 180.0,
       -119.84  * M_PI / 180.0,
        100.0
    };
    GodotEnuProjector projector(origin);

    const auto p = projector.project(origin.lat_rad, origin.lon_rad, 600.0f);
    EXPECT_NEAR(p.x_m,    0.0f, 1e-2f);
    EXPECT_NEAR(p.y_m,  500.0f, 1e-2f);  // 600 m WGS84 - 100 m origin
    EXPECT_NEAR(p.z_m,    0.0f, 1e-2f);
}
