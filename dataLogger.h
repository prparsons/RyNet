#pragma once

#include <thread>
#include <vector>
#include "locksignal.h"
#include "data_types.h"

void printL(const char* sc);

struct ImageLogData {
    time_t timeLast;
    std::string strData;
    float fLat;
    float fLon;
    float fHeading;
    float fPitch;
    //d435i has an accelerometer. So we can figure out camera pitch from that,
    //  though if the drone is accelerating it would skew that measurement. We
    //  can get accel data from drone, as well as actual orientation. Should
    //  be able to use that to correct any error in our pitch calculation
};

class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    void Start();
    void Stop();

    void LogImage(cv::Mat& im, std::string strIdSource);
    void LogMask(float4* pImg, int width, int height);
    void LogString(const char* scData);

private:
    void run(Signal sigImage, StopSignal ss);
    void WriteImageData(cv::Mat& image, ImageLogData& imageData);

    bool bWriteImg;
    bool bWriteMask;

    ThreadMutex mt;
    Signal sigNewData;
    StopSignal ssThread;
    std::thread* pThread;

    cv::Mat image;
    ImageLogData dataImage;
    cv::Mat mask;
    std::vector<char*> listStrings;
    std::map<std::string, ImageLogData> mapImageData;
};
