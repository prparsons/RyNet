#pragma once

#include <thread>
#include <vector>
#include "threadLib/threadLib.hpp"
#include "data_types.h"

void printL(const char* sc);

// // std::tuple<std::shared_ptr<float4>, int, int, int>? on the float4?
// using LogSenderMask = ThreadPipeSender<CameraFrame, ThreadPipeShared<CameraFrame>>;

using LogSender_String = ThreadPipeSender<std::string>;
using LogReceiver_String = ThreadPipeReceiver<std::string>;
using LogSender_Image = ThreadPipeSender<CameraFrame>; //todo this is a duplicate typdef
using LogReceiver_Image = ThreadPipeReceiver<CameraFrame>;

class DataLoggerWriter
{
public:
    DataLoggerWriter(LogSender_String* pStrings, LogSender_Image* pImages, LogSender_Image* pIr, PipeSender_Mask* pMask);
    DataLoggerWriter(DataLoggerWriter& other); // Creates new Sender on same set of pipes

    void LogString(const char* scData);
    void LogImage(const CameraFrame& frm, std::string strData);
    void LogImageIr(const CameraFrame& frm);
    void LogMask(float4* pImg, int iImageId, int width, int height); // Takes over pImg

private:
    std::unique_ptr<LogSender_String> strings;//todo once I verify logging works, all classes with pipes they own/delete should do this?
    std::unique_ptr<LogSender_Image> images;
    std::unique_ptr<LogSender_Image> ir;
    std::unique_ptr<PipeSender_Mask> mask;
};

class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    void Start();
    void Stop();

    DataLoggerWriter* GetLogger();

private:
    void run(Signal sigStop);
    void WriteImageData(CameraFrame& image);
    void SaveImage(std::string strPrefix, CameraFrame& image);

    ThreadMutex mt;
    Signal sigStopThread;
    std::thread* pThread;

    LogReceiver_Image pipeImage;
    LogReceiver_Image pipeIR;
    LogReceiver_String pipeStrings;
    PipeReceiver_Mask pipeMask;
};
