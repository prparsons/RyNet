#include "datalogger.h"

#include <time.h>
#include <stdio.h>
#include <fstream>

using namespace std;

//https://learnopencv.com/getting-started-opencv-cuda-module/
DataLogger gStatus;

DataLogger::DataLogger()
{
}

DataLogger::~DataLogger()
{
    if (pSigImage != nullptr) delete pSigImage;
}

void DataLogger::run(void)
{
    printf("DataLogger Thread\n");

    pSigImage = new Signal(sigNewData); // Deleted in destructor
    time_t timeNow;
    struct tm* pTimeStruct;

    fstream fileLog;
    vector<string> listStringsT;
    char scfileNameText[128];
    timeNow = time(nullptr);
    pTimeStruct = localtime(timeNow);
    snprintf(scFileNameText, 
            127, 
            "log_%d_%d_%d_%d_%d.txt", 
            pTimeStruct->tm_year + 1900, 
            pTimeStruct->tm_mon + 1,
            pTimeStruct->tm_mday,
            pTimeStruct->tm_hour,
            pTimeStruct->tm_min);
    delete pTimeStruct;

    cv::Mat imageWrite;
    bool bWriteImgLocal = false;

    while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    {
        // Wait on signal
        pSigImage->wait();
        printf("statusLoop\n");

        // check text list, if not empty write
        mt.lock();
        listStringsT.assign(listStrings);
        mt.unlock();
        if (listStringsT.size() > 0) {
            fileLog.open(scFileNameText, ios::out);
            if (!fileLog) {
                printf("Failed to open log files\n");
            } else {
                do {
                    string str = listStringsT.back;
                    fileLog << str << endl;
                    listStringsT.pop_back();
                } while (listStringsT.size() > 0);
                fileLog.close();
            }
        }

        // check image list, if not empty, write (it's not a list......should there be a bNewImage flag?)
        mt.lock();
        if (bWriteImg) {
            bWriteImg = false;
            bWriteImgLocal = true;
            imageWrite = image; // memcpy?
        }
        mt.unlock();

        if (bWriteImgLocal) {
            bWriteImgLocal = false;
            timeNow = time(nullptr);
            pTimeStruct = localtime(timeNow);
            char scFileName[128];
            snprintf(scFileName, 
                    127, 
                    "img_%d_%d_%d_%d_%d_%d.jpg", 
                    pTimeStruct->tm_year + 1900, 
                    pTimeStruct->tm_mon + 1,
                    pTimeStruct->tm_mday,
                    pTimeStruct->tm_hour,
                    pTimeStruct->tm_min,
                    pTimeStruct->tm_sec);
            cv::imwrite(scFileName, imageWrite);
            delete pTimeStruct;
        }
    }

    delete pSigImage;
}
/*
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <time.h>

int main(int argc, char * argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    //**************************
    float fFrameTimeAvg = .033;
    clock_t tFrame = clock();
    //**************************

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        imshow(window_name, image);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



*/

void DataLogger::LogImage(cv::Mat& im)
{
    printf("DataLogger::LogImage\n");
    mt.lock();
    image = im.clone();
    mt.unlock();
    if (pSigImage != nullptr) pSigImage->signal(); // this isn't right yet...something copy
}

void DataLogger::LogString(const char* scData)
{
    printf("DataLogger::LogString\n");
    mt.lock();
    string strData(scData)
    listStrings.push_back(strData);
    mt.unlock();
    if (pSigImage != nullptr) pSigImage->signal(); // see above not right comment
}

DataLogger& DataLogger::GetDataLogger()
{
    return gStatus;
}
