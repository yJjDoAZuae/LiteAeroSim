
#define _USE_MATH_DEFINES
#include "Navigation/WGS84.hpp"
#include "Math/math_util.hpp"
#include <math.h>

const double WGS84_Datum::a = 6378137.0; // m, major axis
const double WGS84_Datum::finv = 298.257223563; // inverse of flattening
const double WGS84_Datum::GM = 3.986004418e14; // Gravitational constant
const double WGS84_Datum::omega = 7292115.1467e-11; // earth rotation rate, rad/sec

// derived parameters
const double WGS84_Datum::E = WGS84_Datum::a*sqrt(1.0/WGS84_Datum::finv*(2.0 - 1.0/WGS84_Datum::finv)); // 521854.0084 m, Linear eccentricity
const double WGS84_Datum::e2 = 2.0/WGS84_Datum::finv - 1.0/(WGS84_Datum::finv*WGS84_Datum::finv); // 6.69437999014e-3, First eccentricity squared
const double WGS84_Datum::b = WGS84_Datum::a*(1.0-1.0/WGS84_Datum::finv); // 6356752.314245 m, minor axis;
const double WGS84_Datum::f = 1.0/WGS84_Datum::finv; // flattening
const double WGS84_Datum::e = sqrt(WGS84_Datum::e2); // First eccentricity

void normalizeLatLon(double *latitude_geodetic_rad, double *longitude_rad);

void WGS84_Datum::printJSON() {
    printf("{\n");
    printf("  \"WGS84_Datum\": {\n");
    printf("    \"latitude_rad\": %.12f,\n", latitudeGeodetic_rad());
    printf("    \"longitude_rad\": %.12f,\n", longitude_rad());
    printf("    \"height_m\": %.6f\n", height_WGS84_m());
    printf("  }\n");
    printf("}\n");
}

void WGS84_Datum::setLatitudeGeodetic_rad(double lat) {
    _latitudeGeodetic_rad = lat;
    normalizeLatLon(&_latitudeGeodetic_rad, &_longitude_rad);
}

void WGS84_Datum::setLongitude_rad(double lon) {
    _longitude_rad = lon;
    normalizeLatLon(&_latitudeGeodetic_rad, &_longitude_rad);
}

void WGS84_Datum::setHeight_WGS84_m(float h) {
    // impose a radius limit
    if (h < -WGS84_Datum::b) {
        h = -WGS84_Datum::b;
    }
    _height_WGS84_m = h;
}

Eigen::Vector3d WGS84_Datum::ECEF() const {
    Eigen::Vector3d ecef;
    latLonHeight2ECEF(latitudeGeodetic_rad(), longitude_rad(), height_WGS84_m(), ecef);
    return ecef;
}

Eigen::Quaterniond WGS84_Datum::qne() const {
    Eigen::Quaterniond q_ne;
    latLon2qne(latitudeGeodetic_rad(), longitude_rad(), q_ne);
    return q_ne;
}

Eigen::Vector3d WGS84_Datum::LLH() const {
    Eigen::Vector3d llh;
    llh(0) = latitudeGeodetic_rad();
    llh(1) = longitude_rad();
    llh(2) = height_WGS84_m();
    return llh;
}

Eigen::Matrix3d WGS84_Datum::Cne() const {
    Eigen::Matrix3d Cne;
    latLon2Cne(latitudeGeodetic_rad(), longitude_rad(), Cne);
    return Cne;
}

// this radius is the radius of curvature of a meridian or an ellipse of constant longitude
double WGS84_Datum::meridionalRadius() const {
    double sin_lat = sin(latitudeGeodetic_rad());
    return a * (1 - e2) / pow(1 - e2 * sin_lat * sin_lat, 1.5);
}

// this radius is the forward radius of curvature for an east-west direction
// but not the radius of a circle of latitude
// it is in a plane normal to the meridian and contains the normal vector to the ellipsoid
// I.e. it is tilted with respect to the horizontal plane unless at the equator
double WGS84_Datum::primeVerticalRadius() const {
    double sin_lat = sin(latitudeGeodetic_rad());
    return a / sqrt(1 - e2 * sin_lat * sin_lat);
}

// this radius is the radius of curvature of a meridian or an ellipse of constant longitude
double WGS84_Datum::northRadius() const {
    return meridionalRadius();
}

// this radius is the radius of curvature of a circle of latitude
double WGS84_Datum::eastRadius() const {
    return primeVerticalRadius() * cos(latitudeGeodetic_rad());
}

// radius of curvature in an arbitrary azimuth direction
// https://www.oc.nps.edu/oc2902w/geodesy/radiigeo.pdf
double WGS84_Datum::skewRadius(double azimuth_rad) const {
    double N = primeVerticalRadius();
    double M = meridionalRadius();
    double cos_lat = cos(latitudeGeodetic_rad());

    double sin_az = sin(azimuth_rad);
    double cos_az = cos(azimuth_rad);

    return N*M / (N*cos_az*cos_az + M*sin_az*sin_az);
}

double WGS84_Datum::latitudeRate(double Vnorth) const {
    return Vnorth / (northRadius() + height_WGS84_m());
}

double WGS84_Datum::longitudeRate(double Veast) const {
    double Rmeridional = meridionalRadius() + height_WGS84_m();
    return Veast / (Rmeridional * cos(latitudeGeodetic_rad()));
}

// angular rate of change of the horizon angle due to forward motion over the ellipsoid
// expressed as a positive value
double WGS84_Datum::horizonRate(double Vnorth, double Veast) const {
    double azimuth_rad = atan2(Veast, Vnorth);
    double Rskew = skewRadius(azimuth_rad) + height_WGS84_m();
    double Vhoriz = sqrt(Vnorth*Vnorth + Veast*Veast);
    return Vhoriz / Rskew;
}

// angular rotation rate of NED frame relative to ECEF due to transport over the ellipsoid, expressed in NED frame
Eigen::Vector3d WGS84_Datum::transportRate(double Vnorth, double Veast) const {

    double azimuth_rad = atan2(Veast, Vnorth);

    // transport rate in the local level frame (forward, right, down)
    Eigen::Vector3d omega_transport_LL;
    omega_transport_LL(0) = 0.0; // roll rate (about forward)
    omega_transport_LL(1) = -horizonRate(Vnorth, Veast); // pitch rate (about right)
    omega_transport_LL(2) = longitudeRate(Veast); // yaw rate (about down)

    return Eigen::AngleAxisd(-azimuth_rad, Eigen::Vector3d::UnitZ()) * omega_transport_LL;
}

void WGS84_Datum::setECEF(const Eigen::Vector3d& ecefDatum) {
    double lat_rad;
    float h_m;
    ECEF2LatHeight(ecefDatum, &lat_rad, &h_m);
    setLatitudeGeodetic_rad(lat_rad);
    setHeight_WGS84_m(h_m);
    double lon_rad;
    ECEF2Lon(ecefDatum, &lon_rad);
    setLongitude_rad(lon_rad);
}

void WGS84_Datum::setQne(const Eigen::Quaterniond& qneDatum) {
    double lat_rad;
    double lon_rad;
    qne2LatLon(qneDatum, &lat_rad, &lon_rad);
    setLatitudeGeodetic_rad(lat_rad);
    setLongitude_rad(lon_rad);
}

void WGS84_Datum::setLLH(const Eigen::Vector3d& llhDatum) {
    setLatitudeGeodetic_rad(llhDatum(0));
    setLongitude_rad(llhDatum(1));
    setHeight_WGS84_m(static_cast<float>(llhDatum(2)));
}

void WGS84_Datum::setCne(const Eigen::Matrix3d& CneDatum) {
    double lat_rad;
    double lon_rad;
    Cne2LatLon(CneDatum, &lat_rad, &lon_rad);
    setLatitudeGeodetic_rad(lat_rad);
    setLongitude_rad(lon_rad);
}

// Simple iterative method
void ECEF2LatHeightIterative(const Eigen::Vector3d &ecefDatum, double *latitude_geodetic_rad, float *height_WGS84_m)
{
    double p = sqrt(ecefDatum.x() * ecefDatum.x() + ecefDatum.y() * ecefDatum.y()); // radius from ECEF z axis
    double theta = atan2(ecefDatum.z() * WGS84_Datum::a, p * WGS84_Datum::b);
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    double hgtEst = 0.0;
    double latEst = atan2(ecefDatum.z() + WGS84_Datum::e2 * WGS84_Datum::b * sin_theta * sin_theta * sin_theta,
                                    p - WGS84_Datum::e2 * WGS84_Datum::a * cos_theta * cos_theta * cos_theta);
                            
    // simple iteration    
    for (int i = 0; i < 5; i++) {
        double N = WGS84_Datum::a / sqrt(1 - WGS84_Datum::e2 * sin(latEst) * sin(latEst));
        hgtEst = p / cos(latEst) - N;
        latEst = atan2(ecefDatum.z(), p * (1 - WGS84_Datum::e2 * N / (N + hgtEst)));
    }

    *latitude_geodetic_rad = latEst;
    *height_WGS84_m = static_cast<float>(hgtEst);

}

// Newton-Raphson method
void ECEF2LatHeightNewton(const Eigen::Vector3d &ecefDatum, double *latitude_geodetic_rad, float *height_WGS84_m)
{
    double p = sqrt(ecefDatum.x() * ecefDatum.x() + ecefDatum.y() * ecefDatum.y()); // radius from ECEF z axis
    
    double K0 = 1.0/(1.0 - WGS84_Datum::e2);
    double K = K0;
    double ci;

    double oneMinusE2 = 1.0 - WGS84_Datum::e2;
    double p2 = p*p;
    double Z2 = ecefDatum.z()*ecefDatum.z();

    for (int i = 0; i < 5; i++) {

        ci = pow(p2 + oneMinusE2*Z2*K*K, 1.5)/(WGS84_Datum::a*WGS84_Datum::e2);
        K = 1 + (p2 + oneMinusE2*Z2*K*K*K) / (ci - p2);

    }

    *latitude_geodetic_rad = atan2(ecefDatum.z()*K, p);
    *height_WGS84_m = static_cast<float>(1.0/WGS84_Datum::e2 * (1.0/K - 1.0/K0)*sqrt(p2 + Z2*K*K));

}

void WGS84_Datum::ECEF2LatHeight(const Eigen::Vector3d &ecefDatum, double *latitude_geodetic_rad, float *height_WGS84_m)
{
    ECEF2LatHeightNewton(ecefDatum, latitude_geodetic_rad, height_WGS84_m);
}

void WGS84_Datum::ECEF2Lon(const Eigen::Vector3d &ecefDatum, double *longitude_rad)
{
    *longitude_rad = atan2(ecefDatum.y(), ecefDatum.x());
}

void WGS84_Datum::qne2LatLon(const Eigen::Quaterniond &qne, double *latitude_geodetic_rad, double *longitude_rad)
{
    WGS84_Datum::Cne2LatLon(qne.toRotationMatrix(), latitude_geodetic_rad, longitude_rad);
}

void WGS84_Datum::latLon2qne(double latitude_geodetic_rad, double longitude_rad, Eigen::Quaterniond &q_ne)
{
    Eigen::Matrix3d Cne;
    WGS84_Datum::latLon2Cne(latitude_geodetic_rad, longitude_rad, Cne);
    q_ne = Eigen::Quaterniond(Cne);
}

void WGS84_Datum::Cne2LatLon(const Eigen::Matrix3d &Cne, double *latitude_geodetic_rad, double *longitude_rad)
{
    // flip earth axes to match navigation frame at equator/prime meridian
    Eigen::Matrix3d Cn0e;
    Cn0e << 0, 0, 1,
            0, 1, 0,
           -1, 0, 0;

    Eigen::Matrix3d Cnn0 = Cne * Cn0e.transpose();

    // longitude only is correct for this version
    // Eigen::Vector3d euler = Cnn0.transpose().eulerAngles(2, 1, 0);
    // *latitude_geodetic_rad = -euler.y();
    // *longitude_rad = euler.z();

    Eigen::Vector3d euler = Cnn0.eulerAngles(2, 1, 0);
    *latitude_geodetic_rad = euler.y();
    *longitude_rad = -euler.z();

    normalizeLatLon(latitude_geodetic_rad, longitude_rad);
}

void normalizeLatLon(double *latitude_geodetic_rad, double *longitude_rad)
{
    *latitude_geodetic_rad = MathUtil::wrapToPi(*latitude_geodetic_rad);
    *longitude_rad = MathUtil::wrapToPi(*longitude_rad);

    if (fabs(*latitude_geodetic_rad) > (M_PI / 2.0)) {
        // correct for latitude wrapping
        *latitude_geodetic_rad = MathUtil::wrapToPi(*latitude_geodetic_rad);
        *latitude_geodetic_rad = (*latitude_geodetic_rad > 0) ? (M_PI - *latitude_geodetic_rad) : (-M_PI - *latitude_geodetic_rad);
        *longitude_rad = MathUtil::wrapToPi(*longitude_rad + M_PI);
    }
}

void WGS84_Datum::latLon2Cne(double latitude_geodetic_rad, double longitude_rad, Eigen::Matrix3d &Cne)
{
    Eigen::Matrix3d Cnn0;
    Cnn0 = Eigen::AngleAxisd(latitude_geodetic_rad, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(-longitude_rad, Eigen::Vector3d::UnitX());

    Eigen::Matrix3d Cn0e;
    Cn0e << 0, 0, 1,
            0, 1, 0,
           -1, 0, 0;

    Cne = Cnn0 * Cn0e;
}

void WGS84_Datum::latLonHeight2ECEF(double latitude_geodetic_rad, double longitude_rad, float height_WGS84_m, Eigen::Vector3d &ecefDatum)
{
    double N = WGS84_Datum::a / sqrt(1 - WGS84_Datum::e2 * sin(latitude_geodetic_rad) * sin(latitude_geodetic_rad));

    ecefDatum.x() = (N + height_WGS84_m) * cos(latitude_geodetic_rad) * cos(longitude_rad);
    ecefDatum.y() = (N + height_WGS84_m) * cos(latitude_geodetic_rad) * sin(longitude_rad);
    ecefDatum.z() = (N * (1 - WGS84_Datum::e2) + height_WGS84_m) * sin(latitude_geodetic_rad);
}

Eigen::Quaterniond WGS84_Datum::NED2Nav(const Eigen::Quaterniond& qneDatum)
{
 
    // project ECEF Z axis to Nav frame
    Eigen::Vector3d z_nav = qneDatum.toRotationMatrix() * Eigen::Vector3d::UnitZ();

    // extract wander angle
    double sin_wa = z_nav.cross(Eigen::Vector3d::UnitX()).dot(Eigen::Vector3d::UnitZ());
    double cos_wa = z_nav.dot(Eigen::Vector3d::UnitX());
    double wa = atan2(sin_wa, cos_wa);

    // use wander angle to get rotation from NED to navigation frame
    Eigen::Quaterniond q_ned2nav(Eigen::AngleAxisd(wa, Eigen::Vector3d::UnitZ()));

    return q_ned2nav;

}

Eigen::Quaterniond WGS84_Datum::qne_fix(const Eigen::Quaterniond& qneDatum)
{
 
    Eigen::Quaterniond qne_set = qneDatum;
    Eigen::Quaterniond q_ned2nav;

    qne_set.normalize();
    q_ned2nav = WGS84_Datum::NED2Nav(qne_set);
    qne_set = q_ned2nav * qne_set;
    qne_set.normalize();

    if (qne_set.w() < 0) {
        qne_set.coeffs() = -qne_set.coeffs();
    }

    return qne_set;

}

// gravity magnitude in m/s^2 at the given latitude and height
// TODO: check coefficients
double WGS84_Datum::gravityMagnitude_mps2() const {
    // Somigliana model
    double sin_lat = sin(latitudeGeodetic_rad());
    double sin2_lat = sin_lat * sin_lat;

    double g0 = 9.7803253359 * (1 + 0.00193185265241 * sin2_lat) / sqrt(1 - e2 * sin2_lat);
    double g = g0 - (3.086e-6 - 0.004e-6 * sin2_lat) * height_WGS84_m();

    return g;
}
