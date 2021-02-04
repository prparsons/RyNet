#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <thread>

#include "locksignal.h"

typedef cv::Mat image;
class cameraMulti;
struct point {
    int iXY[2];
};

class cameraFrame_depth:protected rs2::depth_frame {
public:
    cameraFrame_depth(const rs2::depth_frame& _frame);
    ~cameraFrame_depth();

    float operator[](const point& xy) const { return get_distance(xy.iXY[0], xy.iXY[1]); }
    int getMetersPerUnit() const { return get_units(); }

    int getWidth() const { return get_width(); }
    int getHeight() const { return get_height(); }

    void applyDecimationFilter();

private:
    rs2::decimation_filter* pDownsample;
};


class cameraFrame {
public:
    cameraFrame();
    cameraFrame(const cameraFrame& other);
    cameraFrame& operator=(const cameraFrame& other);
    ~cameraFrame();

    image getImageRGB();
    image getImageDepth();
    cameraFrame_depth getImageDepthRaw();
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
    cameraFrame_depth* pDepthRaw;
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
        cameraMulti_REdge = 0x08,

        cameraMulti_DownsampleDepth = 0x10
    };

    static cameraMulti* OpenCamera(cameraType type, cameraFeatures features);
    static bool isCameraTypeAvail(cameraType type);

    int getAvailFeatures() const;

    void runCameraLoop();
    cameraFrame* waitForCamera();
    void run(void* pCallbackFrame, void* pCallbackData);
    StopSignal ss;

protected:
    cameraMulti(cameraType, cameraFeatures);
    cameraMulti(const cameraMulti&);

    void run_RS();

private:
    void (*callbackFrame)(cameraFrame*, void*);
    void* pCallbackData;

    camera_RS* pCameraRS;
    cameraType type;
    cameraFeatures features;
};
