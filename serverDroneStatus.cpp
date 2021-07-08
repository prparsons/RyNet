#include "serverDroneStatus.h"

#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <time.h>
#include <stdio.h>

//https://learnopencv.com/getting-started-opencv-cuda-module/
ServerLoop gStatus;

ServerLoop::ServerLoop()
: mtData(mt)
, pSigImage(nullptr)
{
    printf("ServerLoop\n");
}

ServerLoop::~ServerLoop()
{
    if (pSigImage != nullptr) delete pSigImage;
}

void ServerLoop::run(void)
{
    printf("ServerLoop Thread\n");

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

        clock_t tFrameEnd = clock();
        float fTimeFrameSec = float(tFrameEnd - tFrame) / CLOCKS_PER_SEC;
        tFrame = tFrameEnd;
        fFrameTimeAvg = (0.9 * fFrameTimeAvg) + (.1 * fTimeFrameSec);

        // need to use mutex...copy image here? is copying slow?
        if (image.size == 0) continue;
        writeDetections(image);
        writeSpeed(image);
        writeEquation(image);

        // Update the window with new data
        imshow(window_name, image);
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


}

void ServerLoop::SetImage(cv::Mat& im)
{
    mtData.lock();
    image = im.clone();
    mtData.unlock();
    if (pSigImage != nullptr) pSigImage->signal(); // this isn't right yet...something copy
}

ServerLoop& ServerLoop::GetServerLoopSingleton()
{
    return gStatus;
}

void ServerLoop::SetMaxDriveValues(float _fMaxSteer, float _fMaxSpeed, float _fMaxClimb)
{
    mtData.lock();
    fMaxSpeed = _fMaxSpeed;
    fMaxSteer = _fMaxSteer;
    fMaxClimb = _fMaxClimb;
    mtData.unlock();
}

void ServerLoop::SetSpeed(float _fSpeed)
{
    mtData.lock();
    fSpeed = _fSpeed;
    mtData.unlock();
}

void ServerLoop::SetSteer(float _fSteer)
{
    mtData.lock();
    fSteer = _fSteer;
    mtData.unlock();
}

void ServerLoop::SetClimb(float fClimb)
{

}

void ServerLoop::SetOrientation(Orientation _orient)
{
    mtData.lock();
    orient = _orient;
    mtData.unlock();
}

void ServerLoop::SetEquation(DestinationEquation eq)
{
    mtData.lock();
    equation = eq;
    mtData.unlock();
}

void ServerLoop::SetObstructions(const std::vector<Obstruction>& arObstructions)
{

}

void ServerLoop::AddDetection(DetectionBox bounds)
{
    mtData.lock();
    arObjects.push_back(bounds);
    mtData.unlock();
}

void ServerLoop::ClearDetection()
{
    mtData.lock();
    arObjects.clear();
    mtData.unlock();
}

void ServerLoop::writeDetections(cv::Mat& image)
{
    cv::Scalar rgbRect(0, 255, 0);
    for (int i = 0; i < arObjects.size(); i++) {
        cv::Rect rect(arObjects[i].fLeft, arObjects[i].fTop, arObjects[i].fRight - arObjects[i].fLeft, arObjects[i].fBottom - arObjects[i].fTop);
        cv::rectangle(image, rect, rgbRect);
    }
}

void ServerLoop::writeSpeed(cv::Mat& image)
{
    int font = cv::FONT_HERSHEY_SIMPLEX;
    char cs[512];
    snprintf(cs, 512, "Vel: %.1f   Max[m/s]: %.1f", fSpeed, fMaxSpeed);
    cv::String str = cs;
    cv::Scalar rgbTextOutline(0, 0, 0);
    cv::Scalar rgbText(255, 255, 255);
    cv::Point ptSpeed(20, 40);
    cv::putText(image, str, ptSpeed, font, 1.0, rgbTextOutline, 2);//, LINE_AA);
    cv::putText(image, str, ptSpeed, font, 1.0, rgbText, 1);//, LINE_AA);

    snprintf(cs, 512, "Steer: %.1f   Max[rad/s]: %.1f", fSteer, fMaxSteer);
    str = cs;
    ptSpeed.y = 80;
    cv::putText(image, str, ptSpeed, font, 1.0, rgbTextOutline, 2);//, LINE_AA);
    cv::putText(image, str, ptSpeed, font, 1.0, rgbText, 1);//, LINE_AA);
}

void ServerLoop::writeEquation(cv::Mat& image)
{
    int font = cv::FONT_HERSHEY_SIMPLEX;
    char cs[512];
    // fT _ out of _
    if (equation.bQuadratic) {
        snprintf(cs,
                 512, 
                 "Quadratic: x(t)= %.3ft^2 + %.3ft + %.3f   y(t)= %.3ft^2 + %.3ft + %.3f", 
                 equation.fA, equation.fB, equation.fC,
                 equation.fD, equation.fE, equation.fF);
    } else {
        snprintf(cs,
                 512, 
                 "Radial: x(t)= %.3fsin(t) + %.3fcos(t) + %.3f   y(t)= %.3fsin(t) + %.3fcos(t) + %.3f", 
                 equation.fA, equation.fB, equation.fC,
                 equation.fD, equation.fE, equation.fF);
    }
    cv::String str = cs;
    cv::Scalar rgbTextOutline(0, 0, 0);
    cv::Scalar rgbText(255, 255, 255);
    cv::Point ptSpeed(20, 120);
    cv::putText(image, str, ptSpeed, font, 1.0, rgbTextOutline, 2);//, LINE_AA);
    cv::putText(image, str, ptSpeed, font, 1.0, rgbText, 1);//, LINE_AA);

    snprintf(cs, 512, "t[%.3f - %.3f]: %.3f", equation.fT_min, equation.fT_max, equation.fT);
    str = cs;
    ptSpeed.y = 160;
    cv::putText(image, str, ptSpeed, font, 1.0, rgbTextOutline, 2);//, LINE_AA);
    cv::putText(image, str, ptSpeed, font, 1.0, rgbText, 1);//, LINE_AA);
}

void ServerLoop::updateGPU()
{

}
