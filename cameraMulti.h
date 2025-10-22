#pragma once
// Intel RealSense D435i
//  Depth FOV: 87° × 58°
//  RGB FOV:   69° × 42° 

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <thread>

#include "threadLib/threadLib.hpp"
#include "data_types.h"

class CameraController;

class cameraFrame_depth {
public:
    cameraFrame_depth();
    cameraFrame_depth(cameraFrame_depth& other);
    cameraFrame_depth(const rs2::depth_frame& _frame);
    ~cameraFrame_depth();

    float getMetersPerUnit() const;

    int getWidth() const;
    int getHeight() const;
    const uint16_t* getData() const;
    void applyDecimationFilter();

private:
    rs2::depth_frame* pFrame;
    rs2::decimation_filter* pDownsample;
};

using PipeSenderDepth = ThreadPipeSender<cameraFrame_depth>;
using PipeReceiverDepth = ThreadPipeReceiver<cameraFrame_depth>;
using PipeSenderRGB = ThreadPipeSender<cv::Mat>;
using PipeReceiverRGB = ThreadPipeReceiver<cv::Mat>;

class I_InterruptStop {
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
#define CameraController_LowResDepth 0x40

class CameraController:public I_InterruptStop {
public:
    ~CameraController();

    static CameraController* OpenCamera(uint32_t features, const char* scSimulationFile = nullptr);

    // Must delete returned pointers
    PipeReceiverRGB* GetPipeRGB();
    PipeReceiverRGB* GetPipeIR();
    PipeReceiverDepth* GetPipeDepth();

    void Start();
    void Stop();

    void SignalStop() override;
    void JoinDepth();

protected:
    CameraController(uint32_t features, const char* scSimulationFile);
    void run_RS();
    void run_CV(const char* scGStream, PipeSenderRGB& pipe, Signal* pSigStop);
    void run_simulation(PipeSenderRGB& pipe, Signal* pSigStop);

private:
    cameraFrame_depth* pFrameCurrentDepth;
    rs2_vector FrameCurrentAccel;
    cv::Mat FrameCurrentIR;
    ThreadMutex m;

    camera_RS* pCameraRS;
    uint32_t features;
    char scSimulationFile[512];
    int iSimulationFrameDelay;

    static const int NUM_CAMERA_THREADS = 3;
    std::thread* pthrCamera[NUM_CAMERA_THREADS];
    Signal arSigStop[NUM_CAMERA_THREADS];

    PipeSenderDepth pipeDepth;
    PipeSenderRGB pipeRGB;
    PipeSenderRGB pipeIR;
};
