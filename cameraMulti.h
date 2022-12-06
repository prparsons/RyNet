#pragma once
// Intel RealSense D435i
//  Depth FOV: 87° × 58°
//  RGB FOV:   69° × 42° 

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <thread>

#include "locksignal.h"
#include "data_types.h"

class CameraController;

class cameraFrame_depth:protected rs2::depth_frame {
public:
    cameraFrame_depth(const rs2::depth_frame& _frame);
    ~cameraFrame_depth();

    int getMetersPerUnit() const { return get_units(); }

    int getWidth() const { return get_width(); }
    int getHeight() const { return get_height(); }
    const float* getData() const { return (const float*)get_data(); }
    void applyDecimationFilter();

private:
    rs2::decimation_filter* pDownsample;
};

class camera_RS {
public:
    rs2::pipeline pipe;
    rs2::config cfg;
};

#define CameraController_RGB 0x01
#define CameraController_IR 0x02
#define CameraController_Depth 0x04
#define CameraController_DownsampleDepth 0x08

class CameraController {
public:
    ~CameraController();

    static CameraController* OpenCamera(uint32_t features);

    // Must delete returned pointers
    cameraFrame_depth* GetFrameDepth();
    rs2_vector GetFrameAccel();
    bool GetFrameRGB(cv::Mat& imgCopy);
    bool GetFrameIR(cv::Mat& imgCopy);
    Signal* GetSignalDepth();
    Signal* GetSignalRGB();
    Signal* GetSignalIR();

    void run();
    void JoinDepth();

protected:
    CameraController(uint32_t);
    void run_RS();
    void run_CV(const char* scGStream, cv::Mat* pFrameCurrent, Signal* pSig, StopSignal* pss);

private:
    cameraFrame_depth* pFrameCurrentDepth;
    rs2_vector FrameCurrentAccel;
    cv::Mat FrameCurrentRGB;
    cv::Mat FrameCurrentIR;
    ThreadMutex m;

    camera_RS* pCameraRS;
    uint32_t features;

    std::thread* pthrCamera[3];
    StopSignal ss[3];
    Signal sigDepth;
    Signal sigRGB;
    Signal sigIR;
    
    // wbmode=2 //exposuretimerange=\"30000000 30000000\" aelock=true
    //  ! videoflip method=counterclockwise
    // https://developer.ridgerun.com/wiki/index.php/Raspberry_Pi_HQ_camera_IMX477_Linux_driver_for_Jetson#Example_Pipelines
    // ^That says I should be able to manage 2fps just from the src and don't need the rate limit ...on the
    //  other hand do we want slower than 2fps?
    const char* scGStream_RGB = "nvarguscamerasrc sensor-id=0 gainrange=\"4 4\" ispdigitalgainrange=\"1 1\" ! video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1 ! nvvidconv flip-method=1 ! video/x-raw,format=BGRx ! videoconvert ! videorate ! video/x-raw,framerate=1/1 ! appsink";
    const char* scGStream_RGB_F = "nvarguscamerasrc sensor-id=0 wbmode=%d gainrange=\"%d %d\" ispdigitalgainrange=\"1 1\" exposuretimerange=\"%d %d\" aelock=true ! video/x-raw(memory:NVMM),width=3840,height=2560,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! videorate ! video/x-raw,framerate=2/1 ! appsink";
    const char* scGStream_IR = "stuff ! things ! converterize ! kitchen sinks";
};
