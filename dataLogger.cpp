#include "datalogger.h"

#include <time.h>
#include <stdio.h>

//https://learnopencv.com/getting-started-opencv-cuda-module/
DataLogger gStatus;

DataLogger::DataLogger()
: mtData(mt)
, pSigImage(nullptr)
{
    printf("DataLogger\n");
}

DataLogger::~DataLogger()
{
    if (pSigImage != nullptr) delete pSigImage;
}

void DataLogger::run(void)
{
    printf("DataLogger Thread\n");

    const cv::String window_name = "RyNet Status";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    float fFrameTimeAvg = .033;
    clock_t tFrame = clock();
    Signal sigImage;
    pSigImage = new Signal(sigImage); // Deleted in destructor

    while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    {
        // Wait on signal
        sigImage.wait();
        printf("statusLoop\n");

        // check text list, if not empty write

        // check image list, if not empty, write (it's not a list......should there be a bNewImage flag?)

        // Update the window with new data
        imshow(window_name, image);
    }
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
    std::string strData(scData)
    listStrings.push_back(strData);
    mt.unlock();
    if (pSigImage != nullptr) pSigImage->signal(); // see above not right comment
}

DataLogger& DataLogger::GetDataLogger()
{
    return gStatus;
}
