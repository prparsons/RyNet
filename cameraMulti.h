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

    float getMetersPerUnit() const { return get_units(); }

    int getWidth() const { return get_width(); }
    int getHeight() const { return get_height(); }
    const uint16_t* getData() const { return (const uint16_t*)get_data(); }
    void applyDecimationFilter();

private:
    rs2::decimation_filter* pDownsample;
};

class InterruptStop {
public:
    virtual void SignalStop() = 0;
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
#define CameraController_RGBSimulation 0x10
#define CameraController_Accel 0x20

class CameraController:public InterruptStop {
public:
    ~CameraController();

    static CameraController* OpenCamera(uint32_t features, const char* scSimulationFile = nullptr);

    // Must delete returned pointers
    cameraFrame_depth* GetFrameDepth();
    rs2_vector GetFrameAccel();
    bool GetFrameRGB(cv::Mat& imgCopy);
    bool GetFrameIR(cv::Mat& imgCopy);
    Signal* GetSignalDepth();
    Signal* GetSignalRGB();
    Signal* GetSignalIR();

    void Start();
    void Stop();

    virtual void SignalStop();
    void JoinDepth();

protected:
    CameraController(uint32_t features, const char* scSimulationFile);
    void run_RS();
    void run_CV(const char* scGStream, cv::Mat* pFrameCurrent, Signal* pSig, StopSignal* pss);
    void run_simulation(cv::Mat* pFrameCurrent, Signal* pSig, StopSignal* pss);

private:
    cameraFrame_depth* pFrameCurrentDepth;
    rs2_vector FrameCurrentAccel;
    cv::Mat FrameCurrentRGB;
    cv::Mat FrameCurrentIR;
    ThreadMutex m;

    camera_RS* pCameraRS;
    uint32_t features;
    char scSimulationFile[512];
    int iSimulationFrameDelay;

    static const int NUM_CAMERA_THREADS = 3;
    std::thread* pthrCamera[NUM_CAMERA_THREADS];
    StopSignal ss[NUM_CAMERA_THREADS];
    Signal sigDepth;
    Signal sigRGB;
    Signal sigIR;
};
