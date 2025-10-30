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

class ImageMask {
public:
    ImageMask():pImg(nullptr){}
    ~ImageMask(){if(pImg)cudaFreeHost(pImg);}
    float4* pImg;
    int iImageId;
    int width;
    int height;
};

class ImageLogData {
public:
    ImageLogData();
    ImageLogData(const ImageLogData& other);
    virtual ~ImageLogData(){}

    int GetId() const { return id; }
    float GetLat() const { return fLat; }
    float GetLon() const { return fLon; }
    float GetHeading() const { return fHeading; }
    float GetAltitude() const { return fAltitude; }
    float GetPitch() const { return fPitch; }
    time_t GetTime() const { return timeLast; }
    std::string GetString() const { return strData; }

protected:
    time_t timeLast;
    std::string strData;
    int id;
    float fLat;
    float fLon;
    float fHeading;
    float fAltitude;
    float fPitch;
    //d435i has an accelerometer. So we can figure out camera pitch from that,
    //  though if the drone is accelerating it would skew that measurement. We
    //  can get accel data from drone, as well as actual orientation. Should
    //  be able to use that to correct any error in our pitch calculation
};

class CameraFrame: public ImageLogData {
public:
    CameraFrame();
    
    CameraFrame* DeepCopy() const;
    static CameraFrame* CreateFrame(float fLat, float fLon, float fAlt, time_t time, cv::Mat& source); // Deep copy

    void SetString(std::string strData);
    std::string GetString() const;
    const cv::Mat& GetMatrixConst();
    cv::Mat& GetMatrix();
    const char* GetFilename() const;

private:
    bool bIsDeepCopy;
    mutable ThreadMutex mutex;
    mutable char scFileName[128];
    cv::Mat matrix;
    cv::Mat matrixEmpty; // For returning via GetMatrix if (!bIsDeepCopy)

    CameraFrame(CameraFrame& other);
    CameraFrame& operator=(CameraFrame& other); // Not a deep copy
};

using PipeReceiver_Image = ThreadPipeReceiver<CameraFrame>;
using PipeSender_Image = ThreadPipeSender<CameraFrame>;
using PipeReceiver_ImageData = ThreadPipeReceiver<ImageLogData>;
using PipeSender_ImageData = ThreadPipeSender<ImageLogData>;
using PipeReceiver_Mask = ThreadPipeReceiver<ImageMask>;
using PipeSender_Mask = ThreadPipeSender<ImageMask>;
using PipeReceiver_SegmentData = ThreadPipeReceiver<std::pair<ImageLogData, std::string>>;
using PipeSender_SegmentData = ThreadPipeSender<std::pair<ImageLogData, std::string>>;

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
