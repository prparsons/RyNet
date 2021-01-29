#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <thread>

typedef cv::Mat image;

class cameraMulti;

class cameraFrame {
public:
    cameraFrame();
    cameraFrame(const cameraFrame& other);
    cameraFrame& operator=(const cameraFrame& other);
    ~cameraFrame();

    image getImageRGB();
    image getImageDepth();
    image getImageIR();
    image getImageREdge();

    bool hasRGB() const;
    bool hasDepth() const;
    bool hasIR() const;
    bool hasREdge() const;

private:
    // whatever needed for each type
    cv::Mat* pRGB;
    cv::Mat* pDepth;
    cv::Mat* pIR;
    cv::Mat* pREdge;
    int* piCount;
    friend class cameraMulti;
};

class camera_RS {
public:
    rs2::pipeline pipe;
    rs2::config cfg;
};

class cameraMulti {
public:
    ~cameraMulti();
    enum cameraType {
        cameraMulti_CV,
        cameraMulti_RS };

    enum cameraFeatures {
        cameraMulti_RGB = 0x01,
        cameraMulti_IR = 0x02,
        cameraMulti_Depth = 0x04,
        cameraMulti_REdge = 0x08 };

    static cameraMulti* OpenCamera(cameraType type, cameraFeatures features);
    static bool isCameraTypeAvail(cameraType type);

    int getAvailFeatures() const;

    void runCameraLoop();
    cameraFrame* waitForCamera();
    void operator()(void* pCallbackFrame, void* pCallbackData);

protected:
    cameraMulti(cameraType, cameraFeatures);

    void run_RS();

private:
    void (*callbackFrame)(cameraFrame*, void*);
    void* pCallbackData;

    camera_RS* pCameraRS;
    cameraType type;
    cameraFeatures features;
};
