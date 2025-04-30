#pragma once

#include <arm_neon.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <ctime>
#include <jetson-inference/detectNet.h>
#include <jetson-inference/segNet.h>
#include <jetson-utils/loadImage.h>
#include <vector>

#define PI 3.141592653590
#define PI_2 1.570796326794
#define PIx2 6.283185307180
#define PI_180f 0.017453292f
#define f180_PI 57.29578f
#define RADIUS_EARTH 6371

inline float PointDistanceM(float fLat1, float fLon1, float fLat2, float fLon2)
{
    float fLatDiff = (fLat2 - fLat1) * PI_180f;
    float fLonDiff = (fLon2 - fLon1) * PI_180f;

    fLat1 *= PI_180f;
    fLat2 *= PI_180f;

    float a = pow(sin(fLatDiff / 2), 2) + pow(sin(fLonDiff / 2), 2) * cos(fLat1) * cos(fLat2);
    float c = 2 * asin(sqrt(a));
    return RADIUS_EARTH * c * 1000; // Result was in km, x 1000 for meters
}

inline void PointFromBearing(float fLatCurrent, float fLonCurrent, float fDistanceM, float fBearingDeg, float& fLat, float& fLon)
{
    fLatCurrent *= PI_180f;
    fLonCurrent *= PI_180f;
    float fBearingRad = fBearingDeg * PI_180f;
    float fDistanceKm = fDistanceM * 0.001;
    fLat = asin(sin(fLatCurrent) * cos(fDistanceKm / RADIUS_EARTH) + cos(fLatCurrent) * sin(fDistanceKm / RADIUS_EARTH) * cos(fBearingRad));
    fLon = fLonCurrent;
    fLon += atan2(sin(fBearingRad) * sin(fDistanceKm / RADIUS_EARTH) * cos(fLatCurrent), 
                  cos(fDistanceKm / RADIUS_EARTH) - sin(fLatCurrent) * sin(fLat));
    fLat *= f180_PI;
    fLon *= f180_PI;
}

class CameraFrame {
public:
    CameraFrame();

    CameraFrame& operator=(cv::Mat& source); // Deep copy
    CameraFrame& operator=(CameraFrame& other); // Not a deep copy
    void DeepCopy(CameraFrame& other);

    int GetId() const { return id; }

    cv::Mat matrix;

private:
    CameraFrame(CameraFrame& other);

    int id;
};
