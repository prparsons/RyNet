#pragma once

#include <thread>
#include <vector>
#include "threadLib.hpp"
#include "data_types.h"

void printL(const char* sc);

struct ImageLogData {
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

class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    void Start();
    void Stop();

    void LogImage(CameraFrame& frm, std::string strIdSource);
    void LogImageIr(CameraFrame& frm);
    void LogMask(float4* pImg, int iImageId, int width, int height);
    void LogString(const char* scData);

private:
    void run(Signal sigImage, StopSignal ss);
    void WriteImageData(cv::Mat& image, ImageLogData& imageData);
    void SaveImage(std::string strPrefix, cv::Mat& image, ImageLogData& imageData);

    bool bWriteImg;
    bool bWriteImgIr;
    bool bWriteMask;

    ThreadMutex mt;
    Signal sigNewData;
    StopSignal ssThread;
    std::thread* pThread;

    cv::Mat image;
    cv::Mat imageIr;
    ImageLogData dataImage;
    ImageLogData dataImageIr;
    ImageLogData dataMask;
    cv::Mat mask;
    std::vector<char*> listStrings;
    std::map<std::string, ImageLogData> mapImageData;
};
