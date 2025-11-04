#pragma once

#ifndef WGS84_HPP
#define WGS84_HPP

// WGS84 ellipsoid quantities and functions

#include <Eigen/Dense>

class WGS84_Datum {

    private:

        float height_WGS84_m;
        double latitudeGeodetic_rad;
        double longitude_rad;

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
            height_WGS84_m(0),
            latitudeGeodetic_rad(0),
            longitude_rad(0)
         {};
        
        ~WGS84_Datum() {};

        // setters and getters
        void setHeight_WGS84_m(float h) { height_WGS84_m = h; }
        float getHeight_WGS84_m() const { return height_WGS84_m; }
        void setLatitudeGeodetic_rad(double lat) { latitudeGeodetic_rad = lat; }
        double getLatitudeGeodetic_rad() const { return latitudeGeodetic_rad; }
        void setLongitude_rad(double lon) { longitude_rad = lon; }
        double getLongitude_rad() const { return longitude_rad; }

        // JSON print
        void printJSON();

        // Eigen getters
        Eigen::Vector3d ECEF();
        Eigen::Quaterniond qne();
        Eigen::Vector3d LLH();
        Eigen::Matrix3d Cne();

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
        void ECEF2LatHeight(const Eigen::Vector3d& ecefDatum, double* latitude_geodetic_rad, float* height_WGS84_m);
        void ECEF2Lon(const Eigen::Vector3d& ecefDatum, double* longitude_rad);

        // to/from navigation frame quaternion
        void qne2LatLon(const Eigen::Quaterniond& q_ne, double* latitude_geodetic_rad, double* longitude_rad);
        void latLon2qne(double latitude_geodetic_rad, double longitude_rad, Eigen::Quaterniond& q_ne);

        // to/from navigation frame direction cosine matrix
        void Cne2LatLon(const Eigen::Matrix3d& Cne, double* latitude_geodetic_rad, double* longitude_rad);
        void latLon2Cne(double latitude_geodetic_rad, double longitude_rad, Eigen::Matrix3d& Cne);

        // datum conversion to ECEF
        void latLonHeight2ECEF(double latitude_geodetic_rad, double longitude_rad, float height_WGS84_m, Eigen::Vector3d& ecef);

};

#endif // WGS84_HPP
