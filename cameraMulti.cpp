#include "cameraMulti.h"

using namespace cv;

cameraFrame::cameraFrame()
: pRGB(nullptr)
, pDepth(nullptr)
, pIR(nullptr)
, pREdge(nullptr)
, piCount(new int(0))
{

}

cameraFrame::cameraFrame(const cameraFrame& other)
{
    (*this) = other;
}

cameraFrame& cameraFrame::operator=(const cameraFrame& other)
{
    pRGB = other.pRGB;
    pDepth = other.pDepth;
    pIR = other.pIR;
    pREdge = other.pREdge;
    piCount = other.piCount;
    (*piCount)++;
}

cameraFrame::~cameraFrame()
{
    if (--(*piCount) == 0) {
        delete pRGB;
        delete pDepth;
        delete pIR;
        delete pREdge;
        delete piCount;
    }
}

cameraMulti* cameraMulti::OpenCamera(cameraType type, cameraFeatures features)
{
    cameraMulti* pReturn = new cameraMulti(type, features);
    if (pReturn->pCameraRS == nullptr) {
        delete pReturn;
        return nullptr;
    }
    return pReturn;
}

cameraMulti::cameraMulti(cameraType type, cameraFeatures features)
: pCameraRS(nullptr)
{
    if (type == cameraMulti_RS) {
        pCameraRS = new camera_RS;
        if (features & cameraMulti_Depth) {
            pCameraRS->cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720);
        }
    }
}

cameraMulti::~cameraMulti()
{
    if (pCameraRS != nullptr) delete pCameraRS;
}

bool cameraFrame::hasRGB() const
{
    return true;
}

bool cameraFrame::hasIR() const
{
    return true;
}

bool cameraFrame::hasDepth() const
{
    return true;
}

bool cameraFrame::hasREdge() const
{
    return false;
}

image cameraFrame::getImageRGB()
{
    return image();
}

image cameraFrame::getImageDepth()
{
    return image();
}

image cameraFrame::getImageIR()
{
    return image();
}

image cameraFrame::getImageREdge()
{
    return image();
}

void cameraMulti::runCameraLoop()
{
    /*
    while (true) {
        cameraFrame* pFrame = nullptr;
        if (callbackFrame != nullptr) {
            callbackFrame(pFrame, pCallbackData);
        }
    }
    */
}

void cameraMulti::operator()(void* pCallbackFrame, void* _pCallbackData)
{
    callbackFrame = (void(*)(cameraFrame*, void*))pCallbackFrame;
    pCallbackData = _pCallbackData;
    runCameraLoop();
}

void cameraMulti::run_RS()
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    pCameraRS->pipe.start();
    bool bDone = false;
    do {
        rs2::frameset data = pCameraRS->pipe.wait_for_frames(); // Wait for next set of frames from the camera
        //data = data.apply_filter(filter);
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        //Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        cameraFrame frame;
        frame.pDepth = new Mat(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        callbackFrame(&frame, pCallbackData);
    } while (!bDone);
}