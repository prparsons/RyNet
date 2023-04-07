#include "planner.h"

#include <string.h>
#include <chrono>

FlightController* pInstance = nullptr;

FlightController::FlightController(DataLogger& _logger, CameraController& _camera)
: logger(_logger)
, camera(_camera)
, pThreadDepth(nullptr)
, pThreadPoint(nullptr)
, socketDesc(-1)
{
    strcpy(scMavLinkAddress, "127.0.0.1");
    memset(&addressMavLink, 0, sizeof(addressMavLink));
}

FlightController* FlightController::GetFlightController(DataLogger& logger, CameraController& camera)
{
    if (pInstance == nullptr) {
        pInstance = new FlightController(logger, camera);
        return pInstance;
    }
    return nullptr;
}

void FlightController::DestroyFlightController()
{
    if (pInstance == nullptr) return;

    if (pInstance->pThreadDepth != nullptr) {
        printL("FlightController::DestroyFlightController Stopping depths");
        pInstance->ssDepth.stop();
        Signal* pSig = pInstance->camera.GetSignalDepth();
        pSig->signal();
        pInstance->pThreadDepth->join();
        delete pSig;
        delete pInstance->pThreadDepth;
        pInstance->pThreadDepth = nullptr;
    }
    if (pInstance->pThreadPoint != nullptr) {
        printL("FlightController::DestroyFlightController Stopping point logger");
        pInstance->ssPtLog.stop();
        pInstance->pThreadPoint->join();
        delete pInstance->pThreadPoint;
        pInstance->pThreadPoint = nullptr;
    }
    printL("FlightController::DestroyFlightController Deleting flight controller");
    delete pInstance;
    pInstance = nullptr;
}

void FlightController::Start(bool bStartDepth, bool bStartPointLogger)
{
    printL("FlightController::Start");
    if (bStartDepth) {
        pThreadDepth = new std::thread(&FlightController::runDepth, this);
    }
    if (bStartPointLogger) {
        pThreadPoint = new std::thread(&FlightController::runPointLogger, this);
    }
}

FlightController::~FlightController()
{
    printL("FlightController::~FlightController");
    if (pThreadDepth != nullptr) {
        printL("FlightController::~FlightController Depth should have been stopped already");
        ssDepth.stop();
        pThreadDepth->join();
        delete pThreadDepth;
    }
    if (pThreadPoint != nullptr) {
        printL("FlightController::~FlightController Point logger should have been stopped already");
        ssPtLog.stop();
        pThreadPoint->join();
        delete pThreadPoint;
    }

    //socket
}

bool FlightController::GetPosition(float fLat, float fLon, int iTimeoutMS)
{
    if (pInstance == nullptr) return false;
    char* sc = pInstance->SendRecXML("<pos_retrieve />", iTimeoutMS);
    if (sc == nullptr) return false;
    int i;
    // string is in format "LAT LON"
    for (i = 0; i < strlen(sc); i++) {
        if (sc[i] = ' ') break;
    }
    sc[i] = 0;
    fLat = atof(sc);
    fLon = atof(sc + i + 1);
    delete[] sc;
    return true;
}

bool FlightController::GetHeading(float fHeading, int iTimeoutMS)
{
    if (pInstance == nullptr) return false;
    char* sc = pInstance->SendRecXML("<heading_retrieve />", iTimeoutMS);
    if (sc == nullptr) return false;
    fHeading = atof(sc);
    delete[] sc;
    return true;
}

bool FlightController::GetMode(char* scMode, int iStrLen)
{
    char* sc = SendRecXML("<route_retrieve />");
    if (sc == nullptr) return false;
    strncpy(scMode, sc, iStrLen);
    delete sc;
    return true;
}

void FlightController::SetDirection(float fHeadingOffsetDeg)
{
    char sc[128];
    snprintf(sc, 128, "<dir>%3.3f</dir>", fHeadingOffsetDeg);
    SendXML(sc);
}

void FlightController::SetDestination(float fHeadingOffsetDeg, float fDistance)
{
    char sc[128];
    // Distance for obstacles is in cm, assuming that's the case for everything
    snprintf(sc, 128, "<dir dist=%6f>%3.3f</dir>", fDistance, fHeadingOffsetDeg);
    SendXML(sc);
}

void FlightController::SetRTL()
{
    SendXML("<rtl />");
}

void FlightController::SetAltitude(float fAltitudeMeters)
{
    char sc[128];
    snprintf(sc, 128, "<altitude_set>%.2f</altitude_set>", fAltitudeMeters);
    SendXML(sc);
}

void FlightController::RestoreRouteNaive()
{
    char sc[128];
    strcpy(sc, "<route_restore />");
    SendXML(sc);
}

void FlightController::RestoreRouteClean()
{
    // The plan is to somehow track where we've been while not on the pre-planned route,
    //  and then when we restore the route we remove any waypoints that have us flying
    //  in the same areas we've already been.
    // So...do we add stuff to this to keep track of where we've been? or leave it to
    //  the python script
    // This needs to because we need to avoid looping on ourselves anyways

    // Simple version: search list for closest point in waypoint list, make that first by
    //  deleting everything before it
    SendXML("<route_restore clean='simple' />");
}

void FlightController::SetServerAddress(const char* scServerAddress)
{
    strncpy(scMavLinkAddress, scServerAddress, 32);
}

void FlightController::SaveRoute()
{
    char sc[128];
    strcpy(sc, "<route_save />");
    SendXML(sc);
}

void FlightController::runDepth()
{
    printL("FlightController::runDepth");
    Signal* pSig = camera.GetSignalDepth();

    while (!ssDepth.shouldStop()) {
        pSig->wait();
        
        cameraFrame_depth* pFrameDepth = camera.GetFrameDepth();
        if (pFrameDepth == nullptr) {
            continue;
        }
        cameraFrame_depth& frameDepth = *pFrameDepth;
        
        //1280x720 /8 = 160x90 (original resolution / decimation filter = final resolution)
        int width = frameDepth.getWidth();
        int height = frameDepth.getHeight();
        const uint16_t* pData = frameDepth.getData();

        int iRow;
        // https://developer.arm.com/architectures/instruction-sets/intrinsics/
        uint16_t uMax = 64000;
        uint16x4_t u4Min = vld1_dup_u16(&uMax);
        uint16x4_t u4;
        // Scan each row, loading the first 8 depths and keeping the min of them
        //***^...only loading 4 depths below...
        for (iRow = 0; iRow < 30; iRow++, pData += width) {
            u4 = vld1_u16(pData); // Load 4 float
            u4Min = vmin_u16(u4, u4Min); // Compares corresponding element in each, returns the min of each
        }
        uint16_t uResultR = vminv_u16(u4Min); // Sums across the vector and returns single float
        for (; iRow < 60; iRow++, pData += width) {
            u4 = vld1_u16(pData);
            u4Min = vmin_u16(u4, u4Min);
        }
        uint16_t uResultM = vminv_u16(u4Min);
        for (; iRow < 90; iRow++, pData += width) {
            u4 = vld1_u16(pData);
            u4Min = vmin_u16(u4, u4Min);
        }
        uint16_t uResultL = vminv_u16(u4Min);

        char sc[128];
        sprintf(sc, "<obs dir='r'>%5f</obs>", uResultR * frameDepth.getMetersPerUnit());
        SendXML(sc);
        sprintf(sc, "<obs dir='m'>%5f</obs>", uResultM * frameDepth.getMetersPerUnit());
        SendXML(sc);
        sprintf(sc, "<obs dir='l'>%5f</obs>", uResultL * frameDepth.getMetersPerUnit());
        SendXML(sc);
        delete pFrameDepth;
    }
    delete pSig;
}

void FlightController::runPointLogger()
{
    printL("FlightController::runPointLogger");
    bool bGuided = false;
    std::vector<coord> vecVisitedPoints;
    coord currentPosition;
    char scMode[32];

    while (!ssPtLog.shouldStop()) {
        if (bGuided) { // In GUIDED
            // Get the current position, add it to the list of visited points
            if (GetPosition(currentPosition.fLatitude, currentPosition.fLongitude)) {
                vecVisitedPoints.push_back(currentPosition);
            }
            // See if we're in guided or auto
            if (GetMode(scMode, 32)) {
                bGuided = strcmp(scMode, "GUIDED") == 0;
            }
            if (!bGuided) {
                // Just switched to guided from auto, so clear visited points
                vecVisitedPoints.clear();
            }
        } else { // in AUTO            
            // See if we're in guided or auto
            if (GetMode(scMode, 32)) {
                bGuided = strcmp(scMode, "GUIDED") == 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
    mtXML.lock();
    if (socketDesc < 0) ConnectMavLink();
    sendto(socketDesc, strMessage, strlen(strMessage), MSG_CONFIRM, (const struct sockaddr*) &addressMavLink, sizeof(addressMavLink));
    mtXML.unlock();
    #endif
}

char* FlightController::SendRecXML(const char* strMessage, int iTimeoutMS)
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
    mtXML.lock();
    if (socketDesc < 0) ConnectMavLink();
    sendto(socketDesc, strMessage, strlen(strMessage), MSG_CONFIRM, (const struct sockaddr*)&addressMavLink, sizeof(addressMavLink));
    char* scResponse = new char[512];
    int iLength = recv(socketDesc, scResponse, 511, MSG_DONTWAIT);
    if (iLength < 0) {
        // It failed, give it __msec then try one more time
        std::this_thread::sleep_for(std::chrono::milliseconds(iTimeoutMS));
        iLength = recv(socketDesc, scResponse, 511, MSG_DONTWAIT);
        if (iLength < 0) {
            // Still failed...give up
            delete[] scResponse;
            mtXML.unlock();
            return nullptr;
        }
    }
    scResponse[iLength] = 0;
    mtXML.unlock();
    return scResponse;
    #endif
}

void FlightController::ConnectMavLink()
{
    printf("FlightController::ConnectMavLink address[%s] port[%d]\n",  scMavLinkAddress, iMavlinkPort);

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
    addressMavLink.sin_addr.s_addr = inet_addr(scMavLinkAddress);
    #endif
}
