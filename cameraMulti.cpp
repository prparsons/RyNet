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
// cameraFrame

cameraFrame::cameraFrame()
: pImg(nullptr)
, pDepthRaw(nullptr)
, piCount(new int(1))
, pM(new ThreadMutex())
, pLock(new ThreadLock(*pM))
{
    printL("cameraFrame::cameraFrame");
}

cameraFrame::cameraFrame(const cameraFrame& other)
{
    printL("cameraFrame::cameraFrame(other)");
    other.pLock->lock();
    pImg = other.pImg;
    piCount = other.piCount;
    pDepthRaw = other.pDepthRaw;
    pM = other.pM;
    pLock = new ThreadLock(*pM);
    (*piCount)++;
    other.pLock->unlock();
}

cameraFrame::~cameraFrame()
{
    printL("cameraFrame::~cameraFrame");
    pLock->lock();
    if (--(*piCount) == 0) {
        delete pImg;
        delete pDepthRaw;
        delete piCount;
        delete pM;
    }
    pLock->unlock();
    delete pLock;
}

image cameraFrame::getImage()
{
    return *pImg;
}

cameraFrame_depth* cameraFrame::getDepthRaw()
{
    return pDepthRaw;
}

// ***********
// CameraController

CameraController* CameraController::OpenCamera(cameraFeatures features)
{
    CameraController* pReturn = new CameraController(features);
    if (pReturn->pCameraRS == nullptr) {
        delete pReturn;
        printf("CameraController didn't have valid pCameraRS - deleted\n");
        return nullptr;
    }
    return pReturn;
}

cameraFrame* CameraController::GetFrameDepth()
{
    m.lock();
    if (pFrameCurrentDepth == nullptr) {
        m.unlock();
        return nullptr;
    }
    cameraFrame* pFrameReturn = new cameraFrame(*pFrameCurrentDepth);
    m.unlock();
    return pFrameReturn;
}

cameraFrame* CameraController::GetFrameRGB()
{
    m.lock();
    if (pFrameCurrentRGB == nullptr) {
        m.unlock();
        return nullptr;
    }
    cameraFrame* pFrameReturn = new cameraFrame(*pFrameCurrentRGB);
    m.unlock();
    return pFrameReturn;
}

cameraFrame* CameraController::GetFrameIR()
{
    m.lock();
    if (pFrameCurrentIR == nullptr) {
        m.unlock();
        return nullptr;
    }
    cameraFrame* pFrameReturn = new cameraFrame(*pFrameCurrentIR);
    m.unlock();
    return pFrameReturn;
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

CameraController::CameraController(cameraFeatures _features)
: pFrameCurrentDepth(nullptr)
, pFrameCurrentRGB(nullptr)
, pFrameCurrentIR(nullptr)
, pCameraRS(nullptr)
, features(_features)
{
    printL("CameraController::CameraController");
    
    if (features & CameraController_Depth) {
        pCameraRS = new camera_RS;
        pCameraRS->cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720);
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
    if (pFrameCurrentRGB != nullptr) delete pFrameCurrentRGB;
    if (pFrameCurrentIR != nullptr) delete pFrameCurrentIR;
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
    if (features & CameraController_Depth) {
        if (pthrCamera[0] == nullptr) {
            pthrCamera[0] = new std::thread(&CameraController::run_RS, this);
        } else {
            printL("CameraController::run realsense already started");
        }
    }
    if (features & CameraController_RGB) {
        if (pthrCamera[1] == nullptr) {
            pthrCamera[1] = new std::thread(&CameraController::run_CV, this, scGStream_RGB, &pFrameCurrentRGB, &sigRGB, &ss[1]);
        } else {
            printL("CameraController::run cv1 already started");
        }
    }
    if (features & CameraController_IR) {
        if (pthrCamera[2] == nullptr) {
            pthrCamera[2] = new std::thread(&CameraController::run_CV, this, scGStream_IR, &pFrameCurrentRGB, &sigIR, &ss[2]);
        } else {
            printL("CameraController::run cv2 already started");
        }
    }
}

void CameraController::run_RS()
{
    printL("CameraController::run_RS");
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map(2); // 2 should give us a white to black color map
    rs2::pipeline_profile pipelineInfo = pCameraRS->pipe.start();
    if (!pipelineInfo) {
        printL("CameraController error");
        return;
    }
    do {
        rs2::frameset data;
        try {
            if (!pCameraRS->pipe.try_wait_for_frames(&data)) {
                printL("cam fail");
                return;
            }; // Wait for next set of frames from the camera
        } catch (rs2::error e) {
            printf("wait fail, tryin 1 more time; %d\n", e.get_type());
            std::this_thread::sleep_for (std::chrono::seconds(1));
            try {
                data = pCameraRS->pipe.wait_for_frames();
            } catch(rs2::error) {
                printL("cam fail retry");
                return;
            }
        }
        printL("newCameraFrame");
        // Create camera frame
        cameraFrame* pNewFrameDepth = new cameraFrame();

        // Save the raw depth. If applying the decimation filter, before/after color map makes a difference in 
        //      how big the image is
        pNewFrameDepth->pDepthRaw = new cameraFrame_depth(data.get_depth_frame());
        if (features & CameraController_DownsampleDepth) {
            pNewFrameDepth->pDepthRaw->applyDecimationFilter();
        }
        rs2::frame depth = ((rs2::frame*)pNewFrameDepth->pDepthRaw)->apply_filter(color_map);

        // Query depth frame size (width and height)
        int w = depth.as<rs2::video_frame>().get_width();
        int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        pNewFrameDepth->pImg = new Mat(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        
        m.lock();
        if (pFrameCurrentDepth != nullptr) delete pFrameCurrentDepth;
        pFrameCurrentDepth = pNewFrameDepth;
        m.unlock();

        sigDepth.signal();

        printL("finished new camera frame");
    } while (!ss[0].shouldStop());
}

void CameraController::run_CV(const char* scGStream, cameraFrame** ppFrameCurrent, Signal* pSig, StopSignal* pss)
{
    printL("CameraController::run_CV");
    Mat frame;

    // Python way of using gstreamer
    // cap = cv2.VideoCapture('nvarguscamerasrc sensor-id=0 wbmode=2 gainrange=\"2 2\" ispdigitalgainrange=\"2 2\" exposuretimerange=\"50000000 50000000\" aelock=true ! video/x-raw(memory:NVMM),width=1920,height=1080,framerate=10/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
    VideoCapture cap(scGStream, cv::CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
        printL("CameraController::run_CV failed to open");
        return;
    }

    while (!pss->shouldStop()) { // We need a separate stop signal for each thread
        cap.read(frame);
        if (frame.empty()) {
            printf("CameraController::run_CV empty frame");
        }
        // Do something with frame
        cameraFrame* pNewFrame = new cameraFrame();
        pNewFrame->pImg = new Mat(frame);
        m.lock(); // Need own mutex lock/whatever?
        if (*ppFrameCurrent != nullptr) delete *ppFrameCurrent;
        *ppFrameCurrent = pNewFrame;
        m.unlock();

        pSig->signal();
    }
    // According to opencv: the camera will be deinitialized automatically in VideoCapture destructor
}
