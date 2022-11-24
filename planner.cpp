#include "planner.h"


FlightController::FlightController(DataLogger& _logger, CameraController& _camera)
: logger(_logger)
, camera(_camera)
, pThreadDepth(nullptr)
, socketDesc(-1)
{
    memset(&addressMavLink, 0, sizeof(addressMavLink));
}

FlightController::~FlightController()
{
    //thread
    //socket
}

void FlightController::SetDirection(float fHeadingOffsetDeg)
{
    char sc[128];
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
        
        cameraFrame* pFrame = camera.GetFrameDepth();
        if (pFrame == nullptr) continue;
        cameraFrame_depth* pFrameDepth = pFrame->getDepthRaw();
        if (pFrameDepth == nullptr) {
            delete pFrame;
            continue;
        }
        cameraFrame_depth& frameDepth = *pFrameDepth;
        frameDepth.applyDecimationFilter();
        
        //1280x720 /8 = 160x90 (original resolution / decimation filter = final resolution)
        int width = frameDepth.getWidth();
        int height = frameDepth.getHeight();
        int iRow;
        const float* pData = frameDepth.getData();
        // https://developer.arm.com/architectures/instruction-sets/intrinsics/
        float32x4_t f4Min = vmovq_n_f32(999999);
        float32x4_t f4;
        // Scan each row, loading the first 8 depths and keeping the min of them
        for (iRow = 0; iRow < 30; iRow++, pData += width) {
            f4 = vld1q_f32(pData); // Load 4 float
            f4Min = vminq_f32(f4, f4Min); // Compares corresponding element in each, returns the min of each
        }
        float32_t fResultR = vminvq_f32(f4Min); // Sums across the vector and returns single float
        for (; iRow < 60; iRow++, pData += width) {
            f4 = vld1q_f32(pData);
            f4Min = vminq_f32(f4, f4Min);
        }
        float32_t fResultM = vminvq_f32(f4Min);
        for (; iRow < 90; iRow++, pData += width) {
            f4 = vld1q_f32(pData);
            f4Min = vminq_f32(f4, f4Min);
        }
        float32_t fResultL = vminvq_f32(f4Min);

        char sc[128];
        sprintf(sc, "<obs dir='r'>%5f</obs>", fResultR);
        SendXML(sc);
        sprintf(sc, "<obs dir='m'>%5f</obs>", fResultM);
        SendXML(sc);
        sprintf(sc, "<obs dir='l'>%5f</obs>", fResultL);
        SendXML(sc);
    }
}


void FlightController::SendXML(const char* strMessage)
{
    #ifdef FLIGHT_CONTROLLER_TCP
    //Send some data
    if(send(socketDesc, strMessage, strlen(strMessage), 0) < 0) {
        printL("SendXML fail, attempting to connect");
        ConnectMavLink();
        if (send(socketDesc, strMessage, strlen(strMessage), 0) < 0) {
            printL("  That didn't work, still failed");
        }
    }
    #else
    if (socketDesc < 0) ConnectMavLink();
    sendto(socketDesc, strMessage, strlen(strMessage), MSG_CONFIRM, (const struct sockaddr*) &addressMavLink, sizeof(addressMavLink));
    #endif
}

char* FlightController::SendRecXML(const char* strMessage)
{
    #ifdef FLIGHT_CONTROLLER_TCP
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
    int iLength = recv(socketDesc, scResponse, sizeof(511), 0)
    if(iLength < 0){
        printL("SendRecXML fail, no/bad response");
    }
    scResponse[iLength] = 0;
    return scResponse;
    #else
    if (socketDesc < 0) ConnectMavLink();
    sendto(socketDesc, strMessage, strlen(strMessage), MSG_CONFIRM, (const struct sockaddr*)&addressMavLink, sizeof(addressMavLink));
    char* scResponse = new char[512];
    int iLength = recvfrom(socketDesc, scResponse, 511, MSG_WAITALL, nullptr, nullptr); 
    scResponse[iLength] = 0;
    return scResponse;
    #endif
}

void FlightController::ConnectMavLink()
{
    #ifdef FLIGHT_CONTROLLER_TCP
    socketDesc = socket(AF_INET, SOCK_STREAM, 0);
    if (socketDesc < 0) {
        printL("FlightController::ConnectMavLink fail socket");
        return;
    }

    addressMavLink.sin_addr.s_addr = inet_addr(scMavLinkAddress);
    addressMavLink.sin_family = AF_INET;
    addressMavLink.sin_port = htons(iMavlinkPort);

    //Connect to remote server
    if (connect(socketDesc, (struct sockaddr *)&addressMavLink, sizeof(addressMavLink)) < 0) {
        printL("FlightController::ConnectMavLink fail connect");
    }
    #else
    socketDesc = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketDesc < 0) {
        printL("FlightController::ConnectMavLink fail dgram");
        return;
    }

    addressMavLink.sin_family = AF_INET; 
    addressMavLink.sin_port = htons(iMavlinkPort); 
    addressMavLink.sin_addr.s_addr = INADDR_ANY;
    #endif
}
