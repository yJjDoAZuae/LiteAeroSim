#pragma once

#ifndef WGS84_HPP
#define WGS84_HPP

// WGS84 ellipsoid quantities and functions

#include <Eigen/Dense>

class WGS84_Datum {

    private:

        float _height_WGS84_m;
        double _latitudeGeodetic_rad;
        double _longitude_rad;

    public:

        // defining parameters
        static const double a; // m, major axis
        static const double finv; // inverse of flattening
        static const double GM; // Gravitational constant
        static const double omega; // earth rotation rate rad/sec

        // derived parameters
        static const double E; // m, Linear eccentricity
        static const double e2; // 6.69437999014e-3; // First eccentricity squared
        static const double b; // 6356752.314245;
        static const double f; // flattening
        static const double e; // First eccentricity

        WGS84_Datum() :
            _height_WGS84_m(0),
            _latitudeGeodetic_rad(0),
            _longitude_rad(0)
         {};
        
        ~WGS84_Datum() {};

        // setters and getters
        void setHeight_WGS84_m(float h);
        float height_WGS84_m() const { return _height_WGS84_m; }
        void setLatitudeGeodetic_rad(double lat);
        double latitudeGeodetic_rad() const { return _latitudeGeodetic_rad; }
        void setLongitude_rad(double lon);
        double longitude_rad() const { return _longitude_rad; }

        double northRadius() const;
        double eastRadius() const;

        double meridionalRadius() const;
        double primeVerticalRadius() const;
        double skewRadius(double azimuth_rad) const;
        double latitudeRate(double Vnorth) const;
        double longitudeRate(double Veast) const;
        double horizonRate(double Vnorth, double Veast) const;

        Eigen::Vector3d transportRate(double Vnorth, double Veast) const;

        // gravity model
        double gravityMagnitude_mps2() const;

        // JSON print
        void printJSON();

        // Eigen getters
        Eigen::Vector3d ECEF() const;
        Eigen::Quaterniond qne() const;
        Eigen::Vector3d LLH() const;
        Eigen::Matrix3d Cne() const;

        // Eigen setters
        void setECEF(const Eigen::Vector3d& ecefDatum);
        void setQne(const Eigen::Quaterniond& qneDatum);
        void setLLH(const Eigen::Vector3d& llhDatum);
        void setCne(const Eigen::Matrix3d& CneDatum);

        // Rotation from NED to Navigation frame, if datum is provided with a wander angle
        static Eigen::Quaterniond NED2Nav(const Eigen::Quaterniond& qneDatum);

        // align qne to north, normalize, and enforce positive scalar part
        static Eigen::Quaterniond qne_fix(const Eigen::Quaterniond& qneDatum);

    private:

        // datum conversion from ECEF
        static void ECEF2LatHeight(const Eigen::Vector3d& ecefDatum, double* latitude_geodetic_rad, float* height_WGS84_m);
        static void ECEF2Lon(const Eigen::Vector3d& ecefDatum, double* longitude_rad);

        // to/from navigation frame quaternion
        static void qne2LatLon(const Eigen::Quaterniond& q_ne, double* latitude_geodetic_rad, double* longitude_rad);
        static void latLon2qne(double latitude_geodetic_rad, double longitude_rad, Eigen::Quaterniond& q_ne);

        // to/from navigation frame direction cosine matrix
        static void Cne2LatLon(const Eigen::Matrix3d& Cne, double* latitude_geodetic_rad, double* longitude_rad);
        static void latLon2Cne(double latitude_geodetic_rad, double longitude_rad, Eigen::Matrix3d& Cne);

        // datum conversion to ECEF
        static void latLonHeight2ECEF(double latitude_geodetic_rad, double longitude_rad, float height_WGS84_m, Eigen::Vector3d& ecef);

};

#endif // WGS84_HPP
