#pragma once

#include <thread>
#include <vector>
#include "locksignal.h"
#include "data_types.h"

void printL(const char* sc);

class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    void run(void);

    void LogImage(cv::Mat& im);
    void LogMask(float4* pImg, int width, int height);
    void LogString(const char* scData);

private:
    void WriteDetections(cv::Mat& image);
    void WriteSpeed(cv::Mat& image);

    bool bWriteImg;
    bool bWriteMask;
    ThreadMutex mt;
    Signal sigNewData;
    StopSignal ss;
    cv::Mat image;
    cv::Mat mask;
    std::vector<std::string> listStrings;
};
