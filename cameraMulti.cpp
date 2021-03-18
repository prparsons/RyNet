#include "cameraMulti.h"

#include "serverDroneStatus.h"

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
    pDepthRaw = other.pDepthRaw;
    (*piCount)++;
}

cameraFrame::~cameraFrame()
{
    if (--(*piCount) == 0) {
        delete pRGB;
        delete pDepth;
        delete pDepthRaw;
        delete pIR;
        delete pREdge;
        delete piCount;
    }
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
// cameraMulti

cameraMulti* cameraMulti::OpenCamera(cameraType type, cameraFeatures features)
{
    cameraMulti* pReturn = new cameraMulti(type, features);
    if (pReturn->pCameraRS == nullptr) {
        delete pReturn;
        printf("cameraMulti didn't have valid pCameraRS - deleted\n");
        return nullptr;
    }
    return pReturn;
}

cameraMulti::cameraMulti(cameraType _type, cameraFeatures _features)
: pCameraRS(nullptr)
, type(_type)
, features(_features)
{
    printf("cameraMulti::cameraMulti\n");
    if (type == cameraMulti_RS) {
        pCameraRS = new camera_RS;
        if (features & cameraMulti_Depth) {
            pCameraRS->cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720);
        }
        if (features & cameraMulti_RGB) {
            pCameraRS->cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720);
        }
    }
}

cameraMulti::~cameraMulti()
{
    if (pCameraRS != nullptr) delete pCameraRS;
}

void cameraMulti::runCameraLoop()
{
    if (type == cameraMulti_RS) {
        run_RS();
    }
}

void cameraMulti::run(void* pCallbackFrame, void* _pCallbackData)
{
    callbackFrame = (void(*)(cameraFrame*, void*))pCallbackFrame;
    pCallbackData = _pCallbackData;
    runCameraLoop();
}

void cameraMulti::run_RS()
{
    printf("cameraMulti::run_RS\n");
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map(2); // 2 should give us a white to black color map
    rs2::pipeline_profile pipelineInfo = pCameraRS->pipe.start();
    if (!pipelineInfo) {
        printf("cameraMulti error\n");
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
                return;
            }
        }
        // Create camera frame
        cameraFrame frame;

        // Save the raw depth. If applying the decimation filter, before/after color map makes a difference in 
        //      how big the image is
        frame.pDepthRaw = new cameraFrame_depth(data.get_depth_frame());
        if (features & cameraMulti_DownsampleDepth) {
            frame.pDepthRaw->applyDecimationFilter();
        }
        rs2::frame depth = ((rs2::frame*)frame.pDepthRaw)->apply_filter(color_map);

        // Query depth frame size (width and height)
        int w = depth.as<rs2::video_frame>().get_width();
        int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        //Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        frame.pDepth = new Mat(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

        // Now handle the rgb
        rs2::frame rgb = data.get_color_frame();
        w = rgb.as<rs2::video_frame>().get_width();
        h = rgb.as<rs2::video_frame>().get_height();
        frame.pRGB = new Mat(Size(w, h), CV_8UC3, (void*)rgb.get_data(), Mat::AUTO_STEP);
        ServerLoop::GetServerLoopSingleton().SetImage(*frame.pRGB);

        callbackFrame(&frame, pCallbackData);
    } while (!ss.shouldStop());
}