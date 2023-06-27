#include "cameraMulti.h"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "dataLogger.h"

    
// wbmode=2 //exposuretimerange=\"30000000 30000000\" aelock=true
//  ! videoflip method=counterclockwise
// https://developer.ridgerun.com/wiki/index.php/Raspberry_Pi_HQ_camera_IMX477_Linux_driver_for_Jetson#Example_Pipelines
// ^That says I should be able to manage 2fps just from the src and don't need the rate limit ...on the
//  other hand do we want slower than 2fps?
// Note about using gstreamer strings from the command line..
//  You can't leave ' in the string or you'll get invalid uri errors.
const char* scGStream_RGB = "nvarguscamerasrc sensor-id=0 gainrange=\"4 16\" ispdigitalgainrange=\"1 1\" ! video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1 ! nvvidconv flip-method=1 ! video/x-raw,format=BGRx ! videoconvert ! videorate ! video/x-raw,framerate=1/1 ! appsink";
const char* scGStream_RGB_F = "nvarguscamerasrc sensor-id=0 wbmode=%d gainrange=\"%d %d\" ispdigitalgainrange=\"1 1\" exposuretimerange=\"%d %d\" aelock=true ! video/x-raw(memory:NVMM),width=3840,height=2560,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! videorate ! video/x-raw,framerate=2/1 ! appsink";
const char* scGStream_IR_file = "v4l2src device=/dev/video4 ! 'video/x-raw' ! nvvidconv ! 'video/x-raw(memory:NVMM)' ! omxh264enc ! 'video/x-h264,stream-format=byte-stream' ! h264parse ! qtmux ! filesinklocation=testFlir.mp4";
const char* scGStream_IR = "v4l2src device=/dev/video4 ! video/x-raw ! videoconvert ! video/x-raw,format=BGR ! appsink";

int iUniqueId = 0;

using namespace cv;

CameraFrame::CameraFrame()
: id(-1)
{

}

CameraFrame& CameraFrame::operator=(Mat& source)
{
    id = iUniqueId++;
    source.copyTo(matrix);
}

CameraFrame& CameraFrame::operator=(CameraFrame& other)
{
    id = other.id;
    matrix = other.matrix;
}

void CameraFrame::DeepCopy(CameraFrame& source)
{
    id = source.id;
    source.matrix.copyTo(matrix);
}

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
    if (pDownsample == nullptr) {
        pDownsample = new rs2::decimation_filter(8);
    }
    *this = apply_filter(*pDownsample).as<rs2::depth_frame>();
}

// ***********
// CameraController

CameraController* CameraController::OpenCamera(uint32_t features, const char* scSimulationFile)
{
    CameraController* pReturn = new CameraController(features, scSimulationFile);
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

bool CameraController::GetFrameRGB(CameraFrame& frmCopy)
{
    bool bReturn = true;
    m.lock();
    if (FrameCurrentRGB.empty()) bReturn = false;
    else frmCopy = FrameCurrentRGB;
    m.unlock();
    return bReturn;
}

bool CameraController::GetFrameIR(CameraFrame& frmCopy)
{
    bool bReturn = true;
    m.lock();
    if (FrameCurrentIR.empty()) bReturn = false;
    else frmCopy = FrameCurrentIR;
    m.unlock();
    return true;
}

Signal CameraController::GetSignalDepth()
{
    return sigDepth;
}

Signal CameraController::GetSignalRGB()
{
    return sigRGB;
}

Signal CameraController::GetSignalIR()
{
    return sigIR;
}

CameraController::CameraController(uint32_t _features, const char* _scSimulationFile)
: pFrameCurrentDepth(nullptr)
, pCameraRS(nullptr)
, features(_features)
, iSimulationFrameDelay(33)
{
    printL("CameraController::CameraController");

    if (_scSimulationFile != nullptr) {
        strncpy(scSimulationFile, _scSimulationFile, 512);
    } else {
        scSimulationFile[0] = 0;
    }
    
    if (features & CameraController_Depth) {
        pCameraRS = new camera_RS;
        pCameraRS->cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    }
    if (features & CameraController_Accel) {
        if (pCameraRS == nullptr) {
            pCameraRS = new camera_RS;
        }
        pCameraRS->cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F); 
        // enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        // Should be able to get gyro value too, more info: https://dev.intelrealsense.com/docs/rs-motion
    }
    if (features & CameraController_RGBSimulation) {
        // not anything to do here...or copy file path?
        if (scSimulationFile[0] == 0) {
            throw std::invalid_argument("No Video file provided");
        }
    } else if (features & CameraController_RGB) {
        // Set up anything here? resolution, framerate, exposure, white balance?
    }

    pthrCamera[0] = nullptr;
    pthrCamera[1] = nullptr;
    pthrCamera[2] = nullptr;
}

CameraController::~CameraController()
{
    printL("CameraController::~CameraController");
    Stop();
    if (pFrameCurrentDepth != nullptr) delete pFrameCurrentDepth;
    if (pCameraRS != nullptr) delete pCameraRS;
}

void CameraController::Start()
{
    printf("CameraController::Start features=%u\n", features);
    if (features & (CameraController_Accel + CameraController_Depth)) {
        printL("CameraController::Start starting depth");
        if (pthrCamera[0] == nullptr) {
            pthrCamera[0] = new std::thread(&CameraController::run_RS, this);
        } else {
            printL("CameraController::Start realsense already started");
        }
    }

    if (features & CameraController_RGBSimulation) {
        printf("CameraController::Start starting simulation using file: %s\n", scSimulationFile);
        if (pthrCamera[1] == nullptr) {
            pthrCamera[1] = new std::thread(&CameraController::run_simulation, this, &FrameCurrentRGB, &sigRGB, &ss[1]);
        } else {
            printL("CameraController::Start pthrCamera[1] already started");
        }
    } else if (features & CameraController_RGB) { // Can't run actual RGB camera if the simulation is enabled
        printL("CameraController::Start starting RGB");
        if (pthrCamera[1] == nullptr) {
            pthrCamera[1] = new std::thread(&CameraController::run_CV, this, scGStream_RGB, &FrameCurrentRGB, &sigRGB, &ss[1]);
        } else {
            printL("CameraController::Start cv1 already started");
        }
    }

    if (features & CameraController_IR) {
        if (pthrCamera[2] == nullptr) {
            pthrCamera[2] = new std::thread(&CameraController::run_CV, this, scGStream_IR, &FrameCurrentIR, &sigIR, &ss[2]);
        } else {
            printL("CameraController::Start cv2 already started");
        }
    }
}

void CameraController::Stop()
{
    printL("CameraController::Stop");
    for (int i = 0; i < NUM_CAMERA_THREADS; i++) {
        if (pthrCamera[i] != nullptr) {
            printf("CameraController::Stop Stopping thread %d\n", i);
            ss[i].stop();
            pthrCamera[i]->join();
            delete pthrCamera[i];
            pthrCamera[i] = nullptr;
        }
    }
}

void CameraController::SignalStop()
{
    ss[0].stop();
}

void CameraController::JoinDepth()
{
    printL("CameraController::JoinDepth");
    if (pthrCamera[0] != nullptr) {
        pthrCamera[0]->join();
        delete pthrCamera[0];
        pthrCamera[0] = nullptr;
    } else printL("CameraController::JoinDepth depth thread was not started");
}

void CameraController::run_RS()
{
    printL("CameraController::run_RS");
    m.lock();
    uint32_t localFeatures = features;
    m.unlock();

    try {
        bool bFrameRetrieved = false;
        //rs2::colorizer color_map(2); // 2 should give us a white to black color map
        rs2::decimation_filter filterDec(8);
        rs2::pipeline_profile pipelineInfo = pCameraRS->pipe.start(pCameraRS->cfg);
        if (!pipelineInfo) {
            printL("CameraController::run_RS error");
            return;
        }
        do {
            rs2::frameset data;
            if (!pCameraRS->pipe.try_wait_for_frames(&data)) {
                printL("CameraController::run_RS cam fail");
                pCameraRS->pipe.stop();
                return;
            }
            // Verify that we got frames
            if (!bFrameRetrieved) {
                printL("CameraController::run_RS frames have been retrieved");
                bFrameRetrieved = true;
            }
            // Verify that we got the correct number of frames
            if ((localFeatures & CameraController_Depth) && (localFeatures & CameraController_Accel)) {
                if (data.size() != 2) {
                    printL("CameraController::run_RS did not have 2 frames");
                    pCameraRS->pipe.stop();
                    return;
                }
            } else {
                if (data.size() != 1) {
                    printL("CameraController::run_RS did not have 1 frame");
                    pCameraRS->pipe.stop();
                    return;
                }
            }

            // Save the raw depth. If applying the decimation filter, before/after color map makes a difference in 
            //      how big the image is
            cameraFrame_depth* pDepthRaw;
            if (localFeatures & CameraController_DownsampleDepth) {
                rs2::frame depthFrame = data.get_depth_frame();
                pDepthRaw = new cameraFrame_depth(filterDec.process(depthFrame));
            } else {
                pDepthRaw = new cameraFrame_depth(data.get_depth_frame());
            }
            // Handle the frames retrieved by type
            if (localFeatures & CameraController_Accel) {
                rs2_vector frameAccel;
                if (data[0].get_profile().stream_type() == RS2_STREAM_ACCEL) frameAccel = data[0].as<rs2::motion_frame>().get_motion_data();
                else frameAccel = data[1].as<rs2::motion_frame>().get_motion_data();
                m.lock();
                FrameCurrentAccel = frameAccel;
                m.unlock();
            } 
            if (localFeatures & CameraController_Depth) {            
                m.lock();
                if (pFrameCurrentDepth != nullptr) delete pFrameCurrentDepth;
                pFrameCurrentDepth = pDepthRaw;
                m.unlock();
            }

            sigDepth.signal();
        } while (!ss[0].shouldStop());
        pCameraRS->pipe.stop();
    } catch (rs2::error e) {
        printL("run_RS failed");
        // If we get an exception, we aren't getting the camera working again (at least I've never
        //  been able too yet...)
    }
}

void CameraController::run_CV(const char* scGStream, Mat* pFrameCurrent, Signal* pSig, StopSignal* pss)
{
    printL("CameraController::run_CV");
    if (pFrameCurrent == nullptr) {
        printL("CameraController::run_CV current fame nullptr");
        return;
    }
    if (pSig == nullptr) {
        printL("CameraController::run_CV Signal nullptr");
        return;
    }
    if (pss == nullptr) {
        printL("CameraController::run_CV StopSignal nullptr");
        return;
    }
    if (scGStream == nullptr) {
        printL("CameraController::run_CV scGStream is null");
        return;
    }

    printL(scGStream);
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

void CameraController::run_simulation(cv::Mat* pFrameCurrent, Signal* pSig, StopSignal* pss)
{
    cv::VideoCapture cap(scSimulationFile);
    if (!cap.isOpened()) {
        printL("CameraController::run_simulation failed to open file");
        throw std::invalid_argument("cv::VideoCapture failed");
        return;
    }
    if (pFrameCurrent == nullptr) {
        printL("CameraController::run_simulation current fame nullptr");
        return;
    }
    if (pSig == nullptr) {
        printL("CameraController::run_simulation Signal nullptr");
        return;
    }
    if (pss == nullptr) {
        printL("CameraController::run_simulation StopSignal nullptr");
        return;
    }
    int iFramesRead = 0;
    // Looks like we can get frame failures before the file ends, so we'll increment uFrameFail in the instance of a
    //  failure, and keep trying until it overflows
    uint8_t uFrameFail = 0;
    while (!pss->shouldStop()) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            if (++uFrameFail == 0) {
                // It's overflowed, so 255 failures, so time to give up
                printL("CameraController::run_simulation no more frames");
                break;
            }
            continue;
        }
        uFrameFail = 0; // Reset this now that we got a frame
        iFramesRead++;
        
        m.lock();
        frame.copyTo(*pFrameCurrent);
        m.unlock();

        pSig->signal();
        std::this_thread::sleep_for(std::chrono::milliseconds(iSimulationFrameDelay));
    }
    printf("CameraController::run_simulation Frames Read: %d\n", iFramesRead);
}
