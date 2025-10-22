#pragma once

#include <thread>
#include <vector>
#include "threadLib/threadLib.hpp"
#include "data_types.h"

void printL(const char* sc);

class ImageLogData {
public:
    cv::Mat matrix;
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

// // std::tuple<std::shared_ptr<float4>, int, int, int>? on the float4?
// using LogSenderMask = ThreadPipeSender<CameraFrame, ThreadPipeShared<CameraFrame>>;

using LogSender_String = ThreadPipeSender<std::string, ThreadPipeShared<std::string>>;
using LogReceiver_String = ThreadPipeReceiver<std::string, ThreadPipeShared<std::string>>;
using LogSender_Image = ThreadPipeSender<ImageLogData, ThreadPipeShared<ImageLogData>>;
using LogReceiver_Image = ThreadPipeReceiver<ImageLogData, ThreadPipeShared<ImageLogData>>;

class DataLoggerWriter
{
public:
    DataLoggerWriter(LogSender_String* pStrings, LogSender_Image* pImages, LogSender_Image* pIr);
    DataLoggerWriter(DataLoggerWriter& other); // Creates new Sender on same set of pipes

    void LogString(const char* scData);
    void LogImage(CameraFrame& frm, std::string strIdSource);
    void LogImageIr(CameraFrame& frm);
    void LogMask(float4* pImg, int iImageId, int width, int height){}

private:
    std::unique_ptr<LogSender_String> strings;
    std::unique_ptr<LogSender_Image> images;
    std::unique_ptr<LogSender_Image> ir;
};

class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    void Start();
    void Stop();

    DataLoggerWriter* GetLogger();

    void LogMask(float4* pImg, int iImageId, int width, int height);

private:
    void run(Signal sigStop);
    void WriteImageData(cv::Mat& image, ImageLogData& imageData);
    void SaveImage(std::string strPrefix, cv::Mat& image, ImageLogData& imageData);

    ThreadMutex mt;
    Signal sigStopThread;
    std::thread* pThread;

    LogReceiver_Image pipeImage;
    LogReceiver_Image pipeIR;
    LogReceiver_String pipeStrings;
    bool bWriteMask;
    ImageLogData dataMask;
    cv::Mat mask;
    std::map<std::string, ImageLogData> mapImageData;
};
