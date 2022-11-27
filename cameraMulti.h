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
    int x;
    int y;
};

class cameraFrame_depth:protected rs2::depth_frame {
public:
    cameraFrame_depth(const rs2::depth_frame& _frame);
    ~cameraFrame_depth();

    float operator[](const point& xy) const { return get_distance(xy.x, xy.y); }
    int getMetersPerUnit() const { return get_units(); }

    int getWidth() const { return get_width(); }
    int getHeight() const { return get_height(); }
    const float* getData() const { return (const float*)get_data(); }
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

    image getImage();
    cameraFrame_depth* getDepthRaw();

private:
    // whatever needed for each type
    cv::Mat* pImg;
    cameraFrame_depth* pDepthRaw;
    int* piCount;
    ThreadMutex* pM;
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

    // Must delete returned pointers
    cameraFrame* GetFrameDepth();
    cameraFrame* GetFrameRGB();
    cameraFrame* GetFrameIR();
    Signal* GetSignalDepth();
    Signal* GetSignalRGB();
    Signal* GetSignalIR();

    void run();

protected:
    CameraController(cameraFeatures);
    void run_RS();
    void run_CV(const char* scGStream, cameraFrame** ppFrameCurrent, Signal* pSig, StopSignal* pss);

private:
    cameraFrame* pFrameCurrentDepth;
    cameraFrame* pFrameCurrentRGB;
    cameraFrame* pFrameCurrentIR;
    ThreadMutex m;

    camera_RS* pCameraRS;
    cameraFeatures features;

    std::thread* pthrCamera[3];
    StopSignal ss[3];
    Signal sigDepth;
    Signal sigRGB;
    Signal sigIR;
    
    const char* scGStream_RGB = "nvarguscamerasrc sensor-id=0 wbmode=2 gainrange=\"4 4\" ispdigitalgainrange=\"1 1\" exposuretimerange=\"30000000 30000000\" aelock=true ! video/x-raw(memory:NVMM),width=3840,height=2560,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! videorate ! video/x-raw,framerate=2/1 ! appsink";
    const char* scGStream_RGB_F = "nvarguscamerasrc sensor-id=0 wbmode=%d gainrange=\"%d %d\" ispdigitalgainrange=\"1 1\" exposuretimerange=\"%d %d\" aelock=true ! video/x-raw(memory:NVMM),width=3840,height=2560,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! videorate ! video/x-raw,framerate=2/1 ! appsink";
    const char* scGStream_IR = "stuff ! things ! converterize ! kitchen sinks";
};
