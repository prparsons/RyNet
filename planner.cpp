#include "planner.h"


FlightController::FlightController(DataLogger& _logger, CameraController& _camera)
: logger(_logger)
, camera(_camera)
, pThreadDepth(nullptr)
{

}

FlightController()::~FlightController()
{
    //thread
    //socket
}

void FlightController::SetDirection(float fHeadingOffsetDeg)
{
    char* sc[128];
    snprintf(sc, 128, "<direction>%3.3f</direction>", fHeadingOffsetDeg);
    SendXML(sc);
}

void FlightController::SetDestination(float fHeadingOffsetDeg, float fDistance)
{
    //?
}

void FlightController::RestoreRouteNaive()
{

}

void FlightController::RestoreRouteClean()
{

}

void FlightController::SaveRoute()
{

}

void FlightController::runDepth()
{
    Signal* pSig = camera.GetSignalDepth();
    while (!ss.shouldStop()) {
        pSig->wait();
        //get camera frame
        cameraFrame* pFrame = camera.GetFrameDepth();
        if (pFrame == nullptr) continue;
        cameraFrame_depth* pFrameDepth = pFrame->getDepthRaw();
        if (pFrameDepth == nullptr) {
            delete pFrame;
            continue;
        }
        pFrameDepth->applyDecimationFilter();
        cameraFrame_depth& frameDepth = *pFrameDepth;
        //1280x720 /8 = 160x90 (original resolution / decimation filter = final resolution)
        int width = frameDepth.getWidth();
        int height = frameDepth.getHeight();
        int iRow;
        const float* pData = frameDepth.get_data();
        float32_t f4Min = vmovq_n_f32(999999);
        float32x4_t f4;
        // Scan each row, loading the first 8 depths and keeping the min of them
        for (iRow = 0; iRow < 30; iRow++, pData += width) {
            f4 = vld1q_f32(pData); // Load 4 float
            f4Min = vminq_f32(f4, f4Min); // Compares corresponding element in each, returns the min of each
            f4 = vld1q_f32(pData + 4);
            f4Min = vminq_f32(f4, f4Min);
        }
        float32_t fResultR = vminvq_f32(f4Min); // Sums across the vector and returns single float
        for (; iRow < 60; iRow++, pData += width) {
            f4 = vld1q_f32(pData);
            f4Min = vminq_f32(f4, f4Min);
            f4 = vld1q_f32(pData + 4);
            f4Min = vminq_f32(f4, f4Min);
        }
        float32_t fResultM = vminvq_f32(f4Min);
        for (; iRow < 90; iRow++, pData += width) {
            f4 = vld1q_f32(pData);
            f4Min = vminq_f32(f4, f4Min);
            f4 = vld1q_f32(pData + 4);
            f4Min = vminq_f32(f4, f4Min);
        }
        float32_t fResultL = vminvq_f32(f4Min);


    }
}


void FlightController::SendXML(const char* strMessage)
{	
	//Send some data
	if(send(socketDesc, strMessage, strlen(strMessage), 0) < 0) {
		printL("SendXML fail, attempting to connect");
        ConnectMavLink();
        if (send(socketDesc, strMessage, strlen(strMessage), 0) < 0) {
            printL("  That didn't work, still failed");
        }
	}
}

char* FlightController::SendRecXML(const char* strMessage)
{	
	//Send some data
	if(send(socketDesc, strMessage, strlen(strMessage), 0) < 0) {
		printL("SendRecXML fail, attempting to connect");
        ConnectMavLink();
        if (send(socketDesc, strMessage, strlen(strMessage), 0) < 0) {
            printL("  That didn't work, still failed");
            return nullptr;
        }
	}
    char* scResponse = new char[512];
    if(recv(socketDesc, scResponse, sizeof(511), 0) < 0){
        printL("SendRecXML fail, no/bad response");
    }

    return scResponse;
}

void FlightController::ConnectMavLink()
{
    socketDesc = socket(AF_INET, SOCK_STREAM, 0);
    if (socketDesc == -1) {
        printL("FlightController::ConnectMavLink fail socket");
    }

    addressMavLink.sin_addr.s_addr = inet_addr(scMavLinkAddress);
    addressMavLink.sin_family = AF_INET;
    addressMavLink.sin_port = htons(iMavlinkPort);

    //Connect to remote server
    if (connect(socketDesc, (struct sockaddr *)&addressMavLink, sizeof(addressMavLink)) < 0) {
        printL("FlightController::ConnectMavLink fail connect");
    }
}

/*

Drive_TwinTrackAdv::Drive_TwinTrackAdv(std::vector<Navigator*>& arNav)
: DrivePlanner(arNav)
, pCurrentRoute(nullptr)
, fDistBeforeChangeDest(1000)
, iDestAvgCursor(0)
, iSpeedAvgCursor(0)
, iOrientationAvgCursor(0)
{
    CameraController::cameraFeatures features = CameraController::cameraFeatures(CameraController::CameraController_RGB | CameraController::CameraController_IR | CameraController::CameraController_Align);
    pCamera = CameraController::OpenCamera(CameraController::CameraController_RS, features);
    if (pCamera == nullptr) {
        printf("Error opening camera\n");
        return;
    }

    try {     
        pthrCamera = new std::thread(&CameraController::run, pCamera, (void*)&twinTrackAdv_CameraCallback, (void*)this);
    } catch (rs2::error e) {
        std::cout << "Camera Error. Type: " << e.get_type() << std::endl;
        std::cout << "Camera Error. Function: " << e.get_failed_function() << std::endl;
        std::cout << "Camera Error. Args: " << e.get_failed_args() << std::endl;

        static int iRetryCount = 0;
        // Try 3 times to restart the camera
        if (iRetryCount++ < 3) {
            std::cout << "Camera Error. Restart attempt: " << iRetryCount << std::endl;
            //figure out if thread is running...if so stop it
            delete pCamera;
            pCamera = CameraController::OpenCamera(CameraController::CameraController_RS, features);
            if (pCamera == nullptr) {
                std::cout << "Camera Error. Unable to create new camera.\n";
            }
        }
    }
}
*/
