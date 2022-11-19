#pragma once
// Intel RealSense D435i
//  Depth FOV: 87° × 58°
//  RGB FOV:   69° × 42° 

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <thread>

#include "locksignal.h"
#include "data_types.h"

class CameraController;
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
    //cameraFrame& operator=(const cameraFrame& other);
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
    Mutex* pM;
    ThreadLock* pLock;
    friend class CameraController;
};

class camera_RS {
public:
    rs2::pipeline pipe;
    rs2::config cfg;
};

//rename CameraController
class CameraController {
public:
    ~CameraController();

    enum cameraFeatures {
        CameraController_RGB = 0x01,
        CameraController_IR = 0x02,
        CameraController_Depth = 0x04,

        CameraController_DownsampleDepth = 0x08
    };

    static CameraController* OpenCamera(cameraFeatures features);

    cameraFrame* GetCurrentFrame();
    Signal* GetSignalRGB();

    void run();

protected:
    CameraController(cameraFeatures);
    void run_RS();

private:
    cameraFrame* pFrameCurrent;
    Mutex m;

    camera_RS* pCameraRS;
    cameraFeatures features;

    std::thread* pthrCamera[3];
    StopSignal ss;
    Signal sigRGB;
    Signal sigDepth;
    Signal sigThermal;
};
