#include "CameraController.h"

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
: pRGB(nullptr)
, pDepth(nullptr)
, pIR(nullptr)
, pREdge(nullptr)
, pDepthRaw(nullptr)
, piCount(new int(1))
, pM(new Mutex)
, pLock(new ThreadLock(*pM))
{

}

cameraFrame::cameraFrame(const cameraFrame& other)
{
    other.pLock->lock();
    pRGB = other.pRGB;
    pDepth = other.pDepth;
    pIR = other.pIR;
    pREdge = other.pREdge;
    piCount = other.piCount;
    pDepthRaw = other.pDepthRaw;
    pM = other.pM;
    pLock = new ThreadLock(*pM);
    (*piCount)++;
    other.pLock->unlock();
}

cameraFrame::~cameraFrame()
{
    pLock->lock();
    if (--(*piCount) == 0) {
        delete pRGB;
        delete pDepth;
        delete pDepthRaw;
        delete pIR;
        delete pREdge;
        delete piCount;
        delete pM;
    }
    pLock->unlock();
    delete pLock;
}

bool cameraFrame::hasRGB() const
{
    return pRGB != nullptr;
}

bool cameraFrame::hasIR() const
{
    return pIR != nullptr;
}

bool cameraFrame::hasDepth() const
{
    return pDepth != nullptr;
}

bool cameraFrame::hasREdge() const
{
    return pREdge != nullptr;
}

image cameraFrame::getImageRGB()
{
    return *pRGB;
}

image cameraFrame::getImageDepth()
{
    return *pDepth;
}

cameraFrame_depth cameraFrame::getImageDepthRaw()
{
    return *pDepthRaw;
}

image cameraFrame::getImageIR()
{
    return *pIR;
}

image cameraFrame::getImageREdge()
{
    return *pREdge;
}

// ***********
// CameraController

CameraController* CameraController::OpenCamera(cameraType type, cameraFeatures features)
{
    CameraController* pReturn = new CameraController(type, features);
    if (pReturn->pCameraRS == nullptr) {
        delete pReturn;
        printf("CameraController didn't have valid pCameraRS - deleted\n");
        return nullptr;
    }
    return pReturn;
}

cameraFrame* CameraController::getCurrentFrame()
{
    m.lock();
    if (pFrameCurrent == nullptr) {
        m.unlock();
        return nullptr;
    }
    cameraFrame* pFrameReturn = new cameraFrame(*pFrameCurrent);
    m.unlock();
    return pFrameCurrent;
}

CameraController::CameraController(cameraFeatures _features)
: pFrameCurrent(nullptr)
, pCameraRS(nullptr)
, features(_features)
{
    printf("CameraController::CameraController\n");
    
    if (features & CameraController_Depth) {
        pCameraRS = new camera_RS;
        pCameraRS->cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720);
    }

    pthrCamera[0] = nullptr;
    pthrCamera[1] = nullptr;
    pthrCamera[2] = nullptr;
}

CameraController::~CameraController()
{
    if (pCameraRS != nullptr) delete pCameraRS;
    if (pFrameCurrent != nullptr) delete pFrameCurrent;
    if (pthrCamera[0] != nullptr) {
        printf("~CameraController stopping realsense thread");
        pthrCamera[0]->join();
        delete pthrCamera[0];
    }
}

void CameraController::run()
{
    if (features & CameraController_Depth) {
        if (pthrCamera[0] == nullptr) {
            pthrCamera[0] = new std::thread(&CameraController::run_RS, this);
        } else {
            printf("CameraController::run realsense already started");
        }
    }
}

void CameraController::run_RS()
{
    printf("CameraController::run_RS\n");
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map(2); // 2 should give us a white to black color map
    rs2::pipeline_profile pipelineInfo = pCameraRS->pipe.start();
    if (!pipelineInfo) {
        printf("CameraController error\n");
        return;
    }
    do {
        rs2::frameset data;
        try {
            if (!pCameraRS->pipe.try_wait_for_frames(&data)) {
                printf("cam fail\n");
                return;
            }; // Wait for next set of frames from the camera
        } catch (rs2::error e) {
            printf("wait fail, tryin 1 more time; %d\n", e.get_type());
            std::this_thread::sleep_for (std::chrono::seconds(1));
            try {
                data = pCameraRS->pipe.wait_for_frames();
            } catch(rs2::error) {
                printf("cam fail retry\n");
                return;
            }
        }
        printf("newCameraFrame\n");
        // Create camera frame
        cameraFrame frame;

        // Save the raw depth. If applying the decimation filter, before/after color map makes a difference in 
        //      how big the image is
        frame.pDepthRaw = new cameraFrame_depth(data.get_depth_frame());
        if (features & CameraController_DownsampleDepth) {
            frame.pDepthRaw->applyDecimationFilter();
        }
        rs2::frame depth = ((rs2::frame*)frame.pDepthRaw)->apply_filter(color_map);

        // Query depth frame size (width and height)
        int w = depth.as<rs2::video_frame>().get_width();
        int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        frame.pDepth = new Mat(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

        // Now handle the rgb
        rs2::frame rgb = data.get_color_frame();
        w = rgb.as<rs2::video_frame>().get_width();
        h = rgb.as<rs2::video_frame>().get_height();
        frame.pRGB = new Mat(Size(w, h), CV_8UC3, (void*)rgb.get_data(), Mat::AUTO_STEP);
        
        // printf("status monitor new frame\n");
        m.lock();
        if (pFrameCurrent != nullptr) delete pFrameCurrent;
        pFrameCurrent = new cameraFrame(frame);
        m.unlock();

        sigDepth.signal();

        printf("finished new camera frame\n");
    } while (!ss.shouldStop());
}
