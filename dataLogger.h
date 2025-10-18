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

// // std::tuple<CameraFrame, std::string>
// using LogSenderImage = ThreadPipeSender<CameraFrame, ThreadPipeShared<CameraFrame>>;
// // std::tuple<std::shared_ptr<float4>, int, int, int>? on the float4?
// using LogSenderMask = ThreadPipeSender<CameraFrame, ThreadPipeShared<CameraFrame>>;

using LogSender_String = ThreadPipeSender<std::string, ThreadPipeShared<std::string>>;
using LogReceiver_String = ThreadPipeReceiver<std::string, ThreadPipeShared<std::string>>;
using LogSender_Image = ThreadPipeSender<
                            std::pair<CameraFrame, ImageLogData>, 
                            ThreadPipeShared< std::pair<CameraFrame, ImageLogData> >
                        >;
using LogReceiver_Image = ThreadPipeReceiver<
                        std::pair<CameraFrame, ImageLogData>, 
                        ThreadPipeShared< std::tuple<CameraFrame, ImageLogData> >
                    >;

class DataLoggerWriter
{
public:
    DataLoggerWriter(LogReceiver_String* pStrings, LogReceiver_Image* pIr, LogReceiver_Image* pImages);
    DataLoggerWriter(const DataLoggerWriter& other); // Creates new Sender on same set of pipes

    void LogString(const char* scData);
    void LogImage(CameraFrame& frm, std::string strIdSource);
    void LogImageIr(CameraFrame& frm);
    // void LogMask(float4* pImg, int iImageId, int width, int height);

private:
    std::unique_ptr<LogReceiver_String> strings;
    std::unique_ptr<LogReceiver_Image> images;
    std::unique_ptr<LogReceiver_Image> ir;
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
    void run(StopSignal ss);
    void WriteImageData(cv::Mat& image, ImageLogData& imageData);
    void SaveImage(std::string strPrefix, cv::Mat& image, ImageLogData& imageData);

    ThreadMutex mt;
    StopSignal ssThread;
    std::thread* pThread;

    LogReceiver_Image pipeImage
    LogReceiver_Image pipeIR;
    LogReceiver_String pipeStrings;
    bool bWriteMask;
    ImageLogData dataMask;
    cv::Mat mask;
    std::map<std::string, ImageLogData> mapImageData;
};
