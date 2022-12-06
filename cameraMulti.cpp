#include "cameraMulti.h"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "dataLogger.h"

using namespace cv;

cameraFrame_depth::cameraFrame_depth(const rs2::depth_frame& _frame)
: rs2::depth_frame(_frame)
, pDownsample(nullptr)
{

}

cameraFrame_depth::~cameraFrame_depth()
{
    if (pDownsample) {
        delete pDownsample;
    }
}

void cameraFrame_depth::applyDecimationFilter()
{
    pDownsample = new rs2::decimation_filter(8);
    *this = apply_filter(*pDownsample).as<rs2::depth_frame>();
}

// ***********
// CameraController

CameraController* CameraController::OpenCamera(uint32_t features)
{
    CameraController* pReturn = new CameraController(features);
    return pReturn;
}

cameraFrame_depth* CameraController::GetFrameDepth()
{
    m.lock();
    if (pFrameCurrentDepth == nullptr) {
        m.unlock();
        return nullptr;
    }
    cameraFrame_depth* pFrameReturn = new cameraFrame_depth(*pFrameCurrentDepth);
    m.unlock();
    return pFrameReturn;
}

rs2_vector CameraController::GetFrameAccel()
{
    m.lock();
    rs2_vector ret = FrameCurrentAccel;
    m.unlock();
    return ret;
}

bool CameraController::GetFrameRGB(cv::Mat& imgCopy)
{
    bool bReturn = true;
    m.lock();
    if (FrameCurrentRGB.empty()) bReturn = false;
    else FrameCurrentRGB.copyTo(imgCopy);
    m.unlock();
    return bReturn;
}

bool CameraController::GetFrameIR(cv::Mat& imgCopy)
{
    bool bReturn = true;
    m.lock();
    if (FrameCurrentIR.empty()) bReturn = false;
    else FrameCurrentIR.copyTo(imgCopy);
    m.unlock();
    return true;
}

Signal* CameraController::GetSignalDepth()
{
    return new Signal(sigDepth);
}

Signal* CameraController::GetSignalRGB()
{
    return new Signal(sigRGB);
}

Signal* CameraController::GetSignalIR()
{
    return new Signal(sigIR);
}

CameraController::CameraController(uint32_t _features)
: pFrameCurrentDepth(nullptr)
, pCameraRS(nullptr)
, features(_features)
{
    printL("CameraController::CameraController");
    
    if (features & CameraController_Depth) {
        pCameraRS = new camera_RS;
        pCameraRS->cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720);
        pCameraRS->cfg.enable_stream(RS2_STREAM_ACCEL);
    }
    if (features & CameraController_RGB) {
        // Set up anything here? resolution, framerate, exposure, white balance?
    }
    pthrCamera[0] = nullptr;
    pthrCamera[1] = nullptr;
    pthrCamera[2] = nullptr;
}

CameraController::~CameraController()
{
    if (pFrameCurrentDepth != nullptr) delete pFrameCurrentDepth;
    if (pCameraRS != nullptr) delete pCameraRS;
    for (int i = 0; i < 3; i++) {
        if (pthrCamera[i] != nullptr) {
            ss[i].stop();
            pthrCamera[i]->join();
            delete pthrCamera[i];
        }
    }
}

void CameraController::run()
{
    printf("CameraController::run features=%u", features);
    if (features & CameraController_Depth) {
        printL("CameraController::run starting depth");
        if (pthrCamera[0] == nullptr) {
            pthrCamera[0] = new std::thread(&CameraController::run_RS, this);
        } else {
            printL("CameraController::run realsense already started");
        }
    }
    if (features & CameraController_RGB) {
        printL("CameraController::run starting RGB");
        if (pthrCamera[1] == nullptr) {
            pthrCamera[1] = new std::thread(&CameraController::run_CV, this, scGStream_RGB, &FrameCurrentRGB, &sigRGB, &ss[1]);
        } else {
            printL("CameraController::run cv1 already started");
        }
    }
    if (features & CameraController_IR) {
        if (pthrCamera[2] == nullptr) {
            pthrCamera[2] = new std::thread(&CameraController::run_CV, this, scGStream_IR, &FrameCurrentIR, &sigIR, &ss[2]);
        } else {
            printL("CameraController::run cv2 already started");
        }
    }
}

void CameraController::JoinDepth()
{
    if (pthrCamera[0] != nullptr) pthrCamera[0]->join();
    else printL("CameraController::JoinDepth depth thread was not started");
}

void CameraController::run_RS()
{
    printL("CameraController::run_RS");
    try {
        bool bFrameRetrieved = false;
        rs2::colorizer color_map(2); // 2 should give us a white to black color map
        rs2::pipeline_profile pipelineInfo = pCameraRS->pipe.start(pCameraRS->cfg);
        if (!pipelineInfo) {
            printL("CameraController::run_RS error");
            return;
        }
        do {
            rs2::frameset data;
            if (!pCameraRS->pipe.try_wait_for_frames(&data)) {
                printL("CameraController::run_RS cam fail");
                return;
            } else if (bFrameRetrieved) {
                printL("CameraController::run_RS frames have been retrieved");
            }

            // Save the raw depth. If applying the decimation filter, before/after color map makes a difference in 
            //      how big the image is
            cameraFrame_depth* pDepthRaw = new cameraFrame_depth(data.get_depth_frame());
            if (features & CameraController_DownsampleDepth) {
                pDepthRaw->applyDecimationFilter();
            }

            if (data.size() >= 2) {
                rs2_vector frameAccel;
                if (data[0].get_profile().stream_type() == RS2_STREAM_ACCEL) frameAccel = data[0].as<rs2::motion_frame>().get_motion_data();
                else frameAccel = data[1].as<rs2::motion_frame>().get_motion_data();
                m.lock();
                FrameCurrentAccel = frameAccel;
            } else {            
                m.lock();
            }
            if (pFrameCurrentDepth != nullptr) delete pFrameCurrentDepth;
            pFrameCurrentDepth = pDepthRaw;
            m.unlock();

            sigDepth.signal();
        } while (!ss[0].shouldStop());
    } catch (rs2::error e) {
        printL("run_RS failed");
        // If we get an exception, we aren't getting the camera working again (at least I've never
        //  been able too yet...)
    }
}

void CameraController::run_CV(const char* scGStream, Mat* pFrameCurrent, Signal* pSig, StopSignal* pss)
{
    printL("CameraController::run_CV");
    printL(scGStream);
    if (pFrameCurrent == nullptr) {
        printL("run_CV no current frame");
        return;
    }

    VideoCapture* pCap = new VideoCapture(scGStream, cv::CAP_GSTREAMER);
    
    if (!pCap->isOpened()) {
        printL("CameraController::run_CV failed to open");
        return;
    }

    while (!pss->shouldStop()) { // We need a separate stop signal for each thread
        Mat frame;
        pCap->read(frame);
        if (frame.empty()) {
            printL("CameraController::run_CV empty frame");
            // The only mentioned reasons for failure are camera has been disconnected, or there are
            //  no more frames in the video file (not applicable here). So, we'll try reconnecting
            delete pCap;
            pCap = new VideoCapture(scGStream, cv::CAP_GSTREAMER);
            if (!pCap->isOpened()) {
                // Still failed, no point in just giving up, we can keep trying every second or so
                printL("CameraController::run_CV, reconnect attempt failed");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            continue;
        }
        // Do something with frame
        m.lock();
        frame.copyTo(*pFrameCurrent);
        m.unlock();

        pSig->signal();
    }
    // According to opencv: the camera will be deinitialized automatically in VideoCapture destructor
    delete pCap;
}
