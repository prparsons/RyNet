#pragma once

#include <thread>
#include <vector>
#include "locksignal.h"
#include "data_types.h"

class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    void run(void);
    static DataLogger& GetDataLogger();

    void LogImage(cv::Mat& im);
    void LogImage(uchar3* pImg);
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
