#pragma once

#include <thread>
#include <vector>
#include "locksignal.h"
#include "data_types.h"

using namespace std;

class DataLogger {
public:
    DataLogger(const char* scLogFile);
    ~DataLogger();

    void run(void);
    static DataLogger& GetDataLogger();

    void LogImage(cv::Mat& im);
    void LogString(const char* scData);

private:
    void WriteDetections(cv::Mat& image);
    void WriteSpeed(cv::Mat& image);

    Mutex mt;
    cv::Mat image;
    vector<string> listStrings;
};
