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

using LogSender_Image = ThreadPipeSender<
                            std::pair<CameraFrame, ImageLogData>, 
                            ThreadPipeShared< std::pair<CameraFrame, ImageLogData> >
                        >;
using LogReceiver_Image = ThreadPipeReceiver<
                        std::pair<CameraFrame, ImageLogData>, 
                        ThreadPipeShared< std::tuple<CameraFrame, ImageLogData> >
                    >;
using LogSender_String = ThreadPipeSender<std::string, ThreadPipeShared<std::string>>;
using LogReceiver_String = ThreadPipeReceiver<std::string, ThreadPipeShared<std::string>>;

class DataLoggerWriter
{
public:
    DataLoggerWriter(LogReceiver_String* pStrings, LogReceiver_Image* pIr, LogReceiver_Image* pImages);

    void LogImage(CameraFrame& frm, std::string strIdSource);
    void LogImageIr(CameraFrame& frm);
    // void LogMask(float4* pImg, int iImageId, int width, int height);
    void LogString(const char* scData);

private:
    std::unique_ptr<LogReceiver_Image> images;
    std::unique_ptr<LogReceiver_Image> ir;
    std::unique_ptr<LogReceiver_String> strings;
};

class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    void Start();
    void Stop();

    DataLoggerWriter* GetLogger();

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
    LogReceiver_Image pipeImage
    LogReceiver_Image pipeIR;
    ImageLogData dataImage;
    ImageLogData dataImageIr;
    ImageLogData dataMask;
    cv::Mat mask;
    std::vector<char*> listStrings;
    LogReceiver_String pipeStrings;
    std::map<std::string, ImageLogData> mapImageData;
};
