// GeodesyWgs84_test.cpp — Tests for liteaero::geodesy WGS84 transforms.
//
// Verifies geodeticToEcef, ecefOffsetToEnu, enuToEcefOffset against hand-computed
// reference values and via round-trip identities.  Also verifies the curvature
// drop d^2/(2R) — the geometric quantity that drives Issue 7.

#define _USE_MATH_DEFINES
#include "geodesy/Wgs84.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace liteaero::geodesy;

// ---------------------------------------------------------------------------
// Direct geodetic -> ECEF reference values
// ---------------------------------------------------------------------------

TEST(GeodesyWgs84Test, GeodeticToEcef_AtPrimeMeridianEquator_ReturnsSemiMajorAxis) {
    double X = 0.0, Y = 0.0, Z = 0.0;
    geodeticToEcef(0.0, 0.0, 0.0, X, Y, Z);
    EXPECT_NEAR(X, kWgs84A, 1e-6);
    EXPECT_NEAR(Y, 0.0,     1e-6);
    EXPECT_NEAR(Z, 0.0,     1e-6);
}

TEST(GeodesyWgs84Test, GeodeticToEcef_AtNorthPole_ReturnsSemiMinorAxisOnZ) {
    double X = 0.0, Y = 0.0, Z = 0.0;
    geodeticToEcef(M_PI / 2.0, 0.0, 0.0, X, Y, Z);

    // Polar radius b = a * sqrt(1 - e^2)
    const double b = kWgs84A * std::sqrt(1.0 - kWgs84E2);

    EXPECT_NEAR(X, 0.0, 1e-6);
    EXPECT_NEAR(Y, 0.0, 1e-6);
    EXPECT_NEAR(Z, b,   1e-6);
}

TEST(GeodesyWgs84Test, GeodeticToEcef_At90DegEastEquator_ReturnsSemiMajorAxisOnY) {
    double X = 0.0, Y = 0.0, Z = 0.0;
    geodeticToEcef(0.0, M_PI / 2.0, 0.0, X, Y, Z);
    EXPECT_NEAR(X, 0.0,     1e-6);
    EXPECT_NEAR(Y, kWgs84A, 1e-6);
    EXPECT_NEAR(Z, 0.0,     1e-6);
}

TEST(GeodesyWgs84Test, GeodeticToEcef_HeightAboveEllipsoid_AddsRadiallyAtEquator) {
    double X0 = 0.0, Y0 = 0.0, Z0 = 0.0;
    double X1 = 0.0, Y1 = 0.0, Z1 = 0.0;
    geodeticToEcef(0.0, 0.0,    0.0, X0, Y0, Z0);
    geodeticToEcef(0.0, 0.0, 1000.0, X1, Y1, Z1);
    // At the equator and prime meridian, height extends along +X.
    EXPECT_NEAR(X1 - X0, 1000.0, 1e-6);
    EXPECT_NEAR(Y1 - Y0,    0.0, 1e-6);
    EXPECT_NEAR(Z1 - Z0,    0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// ECEF offset -> ENU at a reference
// ---------------------------------------------------------------------------

TEST(GeodesyWgs84Test, EcefOffsetToEnu_PureRadialAtEquator_ProducesPureUp) {
    double east = 0.0, north = 0.0, up = 0.0;
    ecefOffsetToEnu(0.0, 0.0,
                    1.0, 0.0, 0.0,
                    east, north, up);
    EXPECT_NEAR(east,  0.0, 1e-12);
    EXPECT_NEAR(north, 0.0, 1e-12);
    EXPECT_NEAR(up,    1.0, 1e-12);
}

TEST(GeodesyWgs84Test, EcefOffsetToEnu_PureZAtEquator_ProducesPureNorth) {
    double east = 0.0, north = 0.0, up = 0.0;
    ecefOffsetToEnu(0.0, 0.0,
                    0.0, 0.0, 1.0,
                    east, north, up);
    EXPECT_NEAR(east,  0.0, 1e-12);
    EXPECT_NEAR(north, 1.0, 1e-12);
    EXPECT_NEAR(up,    0.0, 1e-12);
}

TEST(GeodesyWgs84Test, EcefOffsetToEnu_PureYAtEquator_ProducesPureEast) {
    double east = 0.0, north = 0.0, up = 0.0;
    ecefOffsetToEnu(0.0, 0.0,
                    0.0, 1.0, 0.0,
                    east, north, up);
    EXPECT_NEAR(east,  1.0, 1e-12);
    EXPECT_NEAR(north, 0.0, 1e-12);
    EXPECT_NEAR(up,    0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Round-trip identities
// ---------------------------------------------------------------------------

TEST(GeodesyWgs84Test, EnuRoundTrip_AtMidLatitude_RestoresOffset) {
    const double ref_lat = 34.4258 * M_PI / 180.0;
    const double ref_lon = -119.84 * M_PI / 180.0;

    const double dX_in = 1234.5;
    const double dY_in = -678.9;
    const double dZ_in = 901.2;

    double east = 0.0, north = 0.0, up = 0.0;
    ecefOffsetToEnu(ref_lat, ref_lon, dX_in, dY_in, dZ_in, east, north, up);

    double dX_out = 0.0, dY_out = 0.0, dZ_out = 0.0;
    enuToEcefOffset(ref_lat, ref_lon, east, north, up, dX_out, dY_out, dZ_out);

    EXPECT_NEAR(dX_out, dX_in, 1e-9);
    EXPECT_NEAR(dY_out, dY_in, 1e-9);
    EXPECT_NEAR(dZ_out, dZ_in, 1e-9);
}

TEST(GeodesyWgs84Test, EnuOffset_RoundTripFromKnownVector_RestoresInputs) {
    const double lat_a = 34.4258 * M_PI / 180.0;
    const double lon_a = -119.84 * M_PI / 180.0;

    double dX = 0.0, dY = 0.0, dZ = 0.0;
    enuToEcefOffset(lat_a, lon_a, 1000.0, 500.0, 100.0, dX, dY, dZ);

    double east = 0.0, north = 0.0, up = 0.0;
    ecefOffsetToEnu(lat_a, lon_a, dX, dY, dZ, east, north, up);

    EXPECT_NEAR(east,  1000.0, 1e-9);
    EXPECT_NEAR(north,  500.0, 1e-9);
    EXPECT_NEAR(up,     100.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Earth curvature drop (the failure mode behind Issue 7)
// ---------------------------------------------------------------------------

TEST(GeodesyWgs84Test, CurvatureDrop_25kmHorizontalAtEquator_MatchesD2Over2R) {
    // Two points on the WGS84 ellipsoid at h=0, separated by ~25 km along the
    // local east axis at the equator.  When the second point is expressed in
    // ENU at the first, the local "up" component must be negative and equal to
    // approximately -d^2/(2R).
    const double R = kWgs84A;
    const double d = 25000.0;
    const double dlon = d / R;

    double X1 = 0.0, Y1 = 0.0, Z1 = 0.0;
    double X2 = 0.0, Y2 = 0.0, Z2 = 0.0;
    geodeticToEcef(0.0, 0.0,  0.0, X1, Y1, Z1);
    geodeticToEcef(0.0, dlon, 0.0, X2, Y2, Z2);

    double east = 0.0, north = 0.0, up = 0.0;
    ecefOffsetToEnu(0.0, 0.0, X2 - X1, Y2 - Y1, Z2 - Z1, east, north, up);

    const double expected_drop = d * d / (2.0 * R);  // ~ 49 m

    EXPECT_NEAR(east,  d,             1.0);
    EXPECT_NEAR(north, 0.0,           1e-6);
    EXPECT_NEAR(up,   -expected_drop, 1.0);
}

// ---------------------------------------------------------------------------
// primeVerticalRadius (the prime-vertical radius of curvature N(lat))
// ---------------------------------------------------------------------------

TEST(GeodesyWgs84Test, PrimeVerticalRadius_AtEquator_EqualsSemiMajorAxis) {
    EXPECT_NEAR(primeVerticalRadius(0.0), kWgs84A, 1e-6);
}

TEST(GeodesyWgs84Test, PrimeVerticalRadius_AtPole_EqualsAOverSqrtOneMinusE2) {
    const double expected = kWgs84A / std::sqrt(1.0 - kWgs84E2);
    EXPECT_NEAR(primeVerticalRadius(M_PI / 2.0), expected, 1e-6);
}
