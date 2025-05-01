#include "planner.h"

#include <string.h>
#include <chrono>
#include <openssl/evp.h>
#include <openssl/err.h>


EVP_PKEY* hmacCreateKey(const char* _scKey);
bool hmacMessage(std::string strMessage, std::string strKey, int iSalt, char** scHmac, char** scSaltAndMessageOut);

FlightController* pInstance = nullptr;

CoordNode::CoordNode(float _fLat, float _fLon)
: fLat(_fLat)
, fLon(_fLon)
, nw(nullptr)
, ne(nullptr)
, se(nullptr)
, sw(nullptr)
{
}

CoordNode::~CoordNode()
{
    DestroyChildren();
}

float CoordNode::GetDistance(float fLatQuery, float fLonQuery) const
{
    return 1.0;
}

const CoordNode* CoordNode::GetClosest(float fLatQuery, float fLonQuery, float& fDistance) const
{
    fDistance = GetDistance(fLatQuery, fLonQuery);
    float fClosestChild = 9999999;
    const CoordNode* pNode = nullptr;

    if (fLatQuery > fLat) {
        // It's north of this node center
        if (fLonQuery > fLon) {
            // It's west of this node center
            pNode = nw->GetClosest(fLatQuery, fLonQuery, fClosestChild);
            
        } else {
            // It's east of this node center
            pNode = ne->GetClosest(fLatQuery,fLonQuery, fClosestChild);
        }
    } else {
        // It's south of this node center
        if (fLonQuery > fLon) {
            // It's west of this node center
            pNode = sw->GetClosest(fLatQuery, fLonQuery, fClosestChild);
        } else {
            // It's east of this node center
            pNode = se->GetClosest(fLatQuery, fLonQuery, fClosestChild);
        }
    }

    if (pNode != nullptr) {
        if (fClosestChild < fDistance) {
            fDistance = fClosestChild;
            return pNode;
        }
    }
    return this;
}

void CoordNode::InsertPoint(float fLatNew, float fLonNew)
{
  if (fLatNew > fLat) {
        // It's north of this node center
        if (fLonNew > fLon) {
            // It's west of this node center
            if (nw == nullptr) {
                nw = new CoordNode(fLatNew, fLonNew);
            } else {
                nw->InsertPoint(fLatNew, fLonNew);
            }
        } else {
            // It's east of this node center
            if (ne == nullptr) {
                ne = new CoordNode(fLatNew, fLonNew);
            } else {
                ne->InsertPoint(fLatNew, fLonNew);
            }
        }
    } else {
        // It's south of this node center
        if (fLonNew > fLon) {
            // It's west of this node center
            if (sw == nullptr) {
                sw = new CoordNode(fLatNew, fLonNew);
            } else {
                sw->InsertPoint(fLatNew, fLonNew);
            }
        } else {
            // It's east of this node center
            if (se == nullptr) {
                se = new CoordNode(fLatNew, fLonNew);
            } else {
                se->InsertPoint(fLatNew, fLonNew);
            }
        }
    }
}

void CoordNode::ResetPosition(float _fLat, float _fLon)
{
    DestroyChildren();
    fLat = _fLat;
    fLon = _fLon;
}

void CoordNode::DestroyChildren()
{
    if (ne != nullptr) {
        delete ne;
        ne = nullptr;
    }
    if (se != nullptr) {
        delete se;
        se = nullptr;
    }
    if (sw != nullptr) {
        delete sw;
        sw = nullptr;
    }
    if (nw != nullptr) {
        delete nw;
        nw = nullptr;
    }
}

void CoordNode::Print() const
{
    printf("{%.3f,%.3f\nNE:", fLat, fLon);
    if (ne != nullptr) ne->Print();
    else printf("nullptr\n");
    
    printf("SE:");
    if (se != nullptr) se->Print();
    else printf("nullptr\n");
    
    printf("SW:");
    if (sw != nullptr) sw->Print();
    else printf("nullptr\n");
    
    printf("NW:");
    if (nw != nullptr) nw->Print();
    else printf("nullptr\n");

    printf("}\n");
}


FlightController::FlightController(DataLogger& _logger, CameraController& _camera)
: logger(_logger)
, camera(_camera)
, pThreadDepth(nullptr)
, pThreadPoint(nullptr)
, pThreadIr(nullptr)
, sigCameraDepth(_camera.GetSignalDepth())
, socketDesc(-1)
, iSalt(0)
, iUdpTimeoutUs(100000)
, iMaxDist(-1)
, fHomeLat(0)
, fHomeLon(0)
, iMissionSpeedMS(8)
, iExtraGuidedDistance(10)
, iRouteCleanType(0)
{
    scKey[0] = 0;
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
    }
    if (pInstance->pThreadPoint != nullptr) {
        printL("FlightController::DestroyFlightController Stopping point logger");
        pInstance->ssPtLog.stop();
    }
    if (pInstance->pThreadIr != nullptr) {
        printL("FlightController::DestroyFlightController Stopping thermal camera");
        pInstance->ssIr.stop();
    }
    printL("FlightController::DestroyFlightController Deleting flight controller");
    delete pInstance;
    pInstance = nullptr;
}

void FlightController::Start(bool bStartDepth, bool bStartPointLogger, bool bStartIr, int _iMaxDist)
{
    iMaxDist = _iMaxDist;
    Start(bStartDepth, bStartPointLogger, bStartIr);
}

void FlightController::Start(bool bStartDepth, bool bStartPointLogger, bool bStartIr)
{
    printL("FlightController::Start");
    if (bStartDepth) {
        pThreadDepth = new std::thread(&FlightController::runDepth, this, sigCameraDepth, ssDepth);
    }
    if (bStartPointLogger) {
        pThreadPoint = new std::thread(&FlightController::runPointLogger, this, ssPtLog);
    }
    if (bStartIr) {
        pThreadIr = new std::thread(&FlightController::runIr, this, sigIr, ssIr);
    }
}

void FlightController::WaitForTakeoff(int iAltitude, const bool& bAbort)
{
    float fCurrentAltitude = 0;
    char scMode[32];
    scMode[0] = 0;
    bool bTakeoffMessage = true;
    while (fCurrentAltitude < float(iAltitude)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (bAbort) return; // Check abort flag (set by ctrl+c interrupt)

        // Want to wait until AUTO mode has been entered before checking altitude
        if (GetMode(scMode, 31)) {
            if (strcmp("AUTO", scMode) == 0) {
                if (bTakeoffMessage) {
                    bTakeoffMessage = false;
                    printL("Has entered auto");
                    logger.LogString("Has entered auto");
                }
                GetAltitude(fCurrentAltitude);
            }
        }
    }
    printf("Has reached startup altitude %.3f", fCurrentAltitude);
    logger.LogString("Has reached startup altitude");

    float fLat;
    float fLon;
    if (GetPosition(fLat, fLon)) {
        fHomeLat = fLat;
        fHomeLon = fLon;
        printf("Setting Home Pos: %.4f, %.4f\n", fHomeLat, fHomeLon);
    }
}

void FlightController::SetUdpTimout(int iTimeoutUs)
{
    iUdpTimeoutUs = iTimeoutUs;
    if (socketDesc == -1) return;
    // If socketDesc already exists, then we'll set the timeout now
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = iUdpTimeoutUs;
    setsockopt(socketDesc, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
}

void FlightController::SetMissionSpeed(int _iMissionSpeedMS)
{
    iMissionSpeedMS = _iMissionSpeedMS;
}

void FlightController::SetExtraDistance(int _iExtraGuidedDistance)
{
    iExtraGuidedDistance = _iExtraGuidedDistance;
}

void FlightController::SetRouteClean(int iClean)
{
    iRouteCleanType = iClean;
}

FlightController::~FlightController()
{
    printL("FlightController::~FlightController");
    if (pThreadDepth != nullptr) {
        ssDepth.stop();
        if (pThreadDepth->joinable()) pThreadDepth->join();
        delete pThreadDepth;
    }
    if (pThreadPoint != nullptr) {
        ssPtLog.stop();
        if (pThreadPoint->joinable()) pThreadPoint->join();
        delete pThreadPoint;
    }
    if (pThreadIr != nullptr) {
        ssIr.stop();
        if (pThreadIr->joinable()) pThreadIr->join();
        delete pThreadIr;
    }

    //socket
}

bool FlightController::GetPosition(float& fLat, float& fLon)
{
    if (pInstance == nullptr) return false;
    char* sc = pInstance->SendRecXML("<pos_retrieve />");
    if (sc == nullptr) return false;
    int i;
    // string is in format "LAT\nLON"
    for (i = 0; i < strlen(sc); i++) {
        if (sc[i] == '\n') break;
    }
    sc[i] = 0;
    fLat = float(atof(sc));
    fLon = float(atof(sc + i + 1));
    delete[] sc;
    return true;
}

bool FlightController::GetHeading(float& fHeading)
{
    if (pInstance == nullptr) return false;
    char* sc = pInstance->SendRecXML("<heading_retrieve />");
    if (sc == nullptr) return false;
    fHeading = atof(sc);
    delete[] sc;
    return true;
}

bool FlightController::GetSpeed(float& fSpeedMS)
{
    char* sc = SendRecXML("<speed_retrieve />");
    if (sc == nullptr) return false;
    fSpeedMS = atof(sc);
    delete[] sc;
    return true;
}

bool FlightController::GetAltitude(float& fAltitude)
{
    if (pInstance == nullptr) return false;
    char* sc = pInstance->SendRecXML("<altitude_retrieve />");
    if (sc == nullptr) return false;
    fAltitude = atof(sc);
    delete[] sc;
    return true;
}

bool FlightController::GetMode(char* scMode, int iStrLen)
{
    char* sc = SendRecXML("<mode_retrieve />");
    if (sc == nullptr) return false;
    strncpy(scMode, sc, iStrLen);
    delete sc;
    return true;
}

int FlightController::GetBattery()
{
    char* sc = SendRecXML("<battery_retrieve />");
    if (sc == nullptr) return -1;
    int iBattery = atoi(sc);
    delete sc;
    return iBattery;
}

Route* FlightController::GetRoute()
{
    return nullptr;
}

int FlightController::CheckDirectionValid(float fDirection, time_t travelTime)
{
    float fCurrentSpeedMS = 0;
    // Get Current speed in m/s, use that to calculate distance traveled in m
    if (!GetSpeed(fCurrentSpeedMS)) {
        printL("Failed to get speed");
        return -1;
    }
    if (float(iMissionSpeedMS) > fCurrentSpeedMS) fCurrentSpeedMS = float(iMissionSpeedMS);
    float fDistanceM = (fCurrentSpeedMS * travelTime) + iExtraGuidedDistance;
    // Home Location must exist and max distance must have been set
    if ((fHomeLat == 0 && fHomeLon == 0) || iMaxDist == -1) {
        return int(fDistanceM);
    }

    // Get current position
    float fLatCurrent = 0;
    float fLonCurrent = 0;
    if (!GetPosition(fLatCurrent, fLonCurrent)) {
        printL("Failed to get position");
        return -1;
    }


    float fQueryLat = 0;
    float fQueryLon = 0;
    // Find the final position based on lat, long, distance
    PointFromBearing(fLatCurrent, fLonCurrent, fDistanceM, fDirection, fQueryLat, fQueryLon);

    // Now find distance between that and home
    float fDistance = PointDistanceM(fHomeLat, fHomeLon, fQueryLat, fQueryLon);
    if (fDistance < iMaxDist) {
        printf("DirectionChange:\nCurrent: [%.4f,%.4f]; Speed: %.1f; DistTravel: %.1f; Dest: [%.4f,%.4f]; Dest-Home Dist: %.1f\n\n", 
            fLatCurrent, fLonCurrent, fCurrentSpeedMS, fDistanceM, fQueryLat, fQueryLon, fDistance);
        return int(fDistanceM);
    }
    printf("Invalid:\nCurrent: [%.4f,%.4f]; Speed: %.1f; DistTravel: %.1f; Dest: [%.4f,%.4f]; Dest-Home Dist: %.1f\n\n", 
        fLatCurrent, fLonCurrent, fCurrentSpeedMS, fDistanceM, fQueryLat, fQueryLon, fDistance);
    return 0;
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
    snprintf(sc, 128, "<dir dist=\"%6f\">%3.3f</dir>", fDistance, fHeadingOffsetDeg);
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

void FlightController::RestoreRoute()
{
    const char* scRouteString = "<route_restore setauto='true'/>";
    if (iRouteCleanType == 1) {
        scRouteString = "<route_restore setauto='true' clean='simple'/>";
    } else if (iRouteCleanType == 2) {
        scRouteString = "<route_restore setauto='true' clean='2opt'/>";
    }
    SendXML(scRouteString);
}

void FlightController::SetRoute(Route* pNewRoute, bool bSetAuto)
{
    if (pNewRoute == nullptr) return;

    delete pNewRoute;
}

void FlightController::SetServerAddress(const char* scServerAddress)
{
    strncpy(scMavLinkAddress, scServerAddress, 32);
}

void FlightController::SetUdpKey(const char* _scKey)
{
    strncpy(scKey, _scKey, 32);
}

void FlightController::SaveRoute()
{
    char sc[128];
    strcpy(sc, "<route_save />");
    SendXML(sc);
}

void FlightController::runDepth(Signal sig, StopSignal ss)
{
    printL("FlightController::runDepth");

    // https://developer.arm.com/architectures/instruction-sets/intrinsics/
    const uint16_t uMax = 65535;
    const uint16_t uUnderflow = 1;
    const uint16x8_t u8Underflow = vld1q_dup_u16(&uUnderflow);

    while (!ss.shouldStop()) {
        sig.wait(1000);
        
        cameraFrame_depth* pFrameDepth = camera.GetFrameDepth();
        if (pFrameDepth == nullptr) {
            continue;
        }
        cameraFrame_depth& frameDepth = *pFrameDepth;
        
        // The image is in portrait orientation, left side is the top. So for forward distance, we're
        //  looking at the first 4 elements in each row.
        //1280x720 /8 = 160x90 (original resolution / decimation filter = final resolution)
        const int width = frameDepth.getWidth();
        const int height = frameDepth.getHeight();
        const int boundaryRight = height / 3;
        const int boundaryMiddle = boundaryRight + boundaryRight;
        const int boundaryLeft = boundaryMiddle + boundaryRight;
        const uint16_t* pData = frameDepth.getData();
        uint16x8_t u8;

        // Scan each row, loading the first 4 depths and keeping the min of them
        int iRow;
        uint16x8_t u8Min = vld1q_dup_u16(&uMax); // Loads a single value into all 8 of a uint16x8_t
        for (iRow = 0; iRow < boundaryRight; iRow++, pData += width) {
            // 0 - 7
            u8 = vld1q_u16(pData); // Loads 8 values from pData[] into a uint16x8_t
            u8 = vsubq_u16(u8, u8Underflow); // Subtract 1 from each element. Invalid values are 0, so this will underflow them and make them max_uint16
            u8Min = vminq_u16(u8, u8Min); // Compares corresponding element in each, returns the min
            // 8-15
            u8 = vld1q_u16(pData + 8);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min); // Compares corresponding element in each, returns the min
            // 16-23
            u8 = vld1q_u16(pData + 16);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min); // Compares corresponding element in each, returns the min
            // 24-31
            u8 = vld1q_u16(pData + 24);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min); // Compares corresponding element in each, returns the min
        }
        uint16_t uResultRight = vminvq_u16(u8Min); // Compares all 8 values and returns the min into a uint16_t
        u8Min = vld1q_dup_u16(&uMax);
        // Do the same for the middle 30 rows (the center...directly ahead)
        for (; iRow < boundaryMiddle; iRow++, pData += width) {
            // 0 - 7
            u8 = vld1q_u16(pData);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min);
            // 8-15
            u8 = vld1q_u16(pData + 8);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min);
            // 16-23
            u8 = vld1q_u16(pData + 16);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min);
            // 24-31
            u8 = vld1q_u16(pData + 24);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min); // Compares corresponding element in each, returns the min
        }
        uint16_t uResultMiddle = vminvq_u16(u8Min);
        u8Min = vld1q_dup_u16(&uMax);
        // Finally the last 30 rows (left of direction of travel)
        for (; iRow < boundaryLeft; iRow++, pData += width) {
            // 0 - 7
            u8 = vld1q_u16(pData);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min);
            // 8-15
            u8 = vld1q_u16(pData + 8);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min);
            // 16-23
            u8 = vld1q_u16(pData + 16);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min);
            // 24-31
            u8 = vld1q_u16(pData + 24);
            u8 = vsubq_u16(u8, u8Underflow);
            u8Min = vminq_u16(u8, u8Min); // Compares corresponding element in each, returns the min
        }
        uint16_t uResultLeft = vminvq_u16(u8Min);
        // Send each message (use the rs2 provided float to convert to meters * 100cm/m)
        char sc[128];
        sprintf(sc, "<obs><dist dir='left'>%5d</dist><dist dir='center'>%5d</dist><dist dir='right'>%5d</dist></obs>",
            int(uResultLeft * frameDepth.getMetersPerUnit() * 100),
            int(uResultMiddle * frameDepth.getMetersPerUnit() * 100),
            int(uResultRight * frameDepth.getMetersPerUnit() * 100));
        SendXML(sc);

        delete pFrameDepth;
        // Limit distance messages to 20hz
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void FlightController::runPointLogger(StopSignal ss)
{
    printL("FlightController::runPointLogger");
    bool bGuided = false;
    char scMode[32];

    float fLat = 0;
    float fLon = 0;
    GetPosition(fLat, fLon);
    CoordNode startLocation(fLat, fLon);

    while (!ss.shouldStop()) {
        if (bGuided) { // In GUIDED
            // Get the current position, add it to the list of visited points
            if (GetPosition(fLat, fLon)) {
                startLocation.InsertPoint(fLat, fLon);
            }
            // See if we're in guided or auto
            if (GetMode(scMode, 32)) {
                bGuided = strcmp(scMode, "GUIDED") == 0;
            }
            if (!bGuided) {
                // Just switched to guided from auto, so clear visited points
                GetPosition(fLat, fLon); // Doesn't matter if it fails...we still need to reset position
                startLocation.ResetPosition(fLat, fLon);
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

void FlightController::runIr(Signal sig, StopSignal ss)
{
    printL("FlightController::runIr");
    CameraFrame frmIr;
    while (!ss.shouldStop()) {
        sig.wait(1000);
        camera.GetFrameIR(frmIr);
        if (frmIr.matrix.empty()) continue;
        logger.LogImageIr(frmIr);
    }
}

void FlightController::SendXML(const char* _scMessage)
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
    char* scSaltedMessage = nullptr;
    char* scHmac = nullptr;
    char* scHmacMessage = nullptr;
    const char* scMessage = _scMessage;
    int iStrLen = strlen(scMessage);
    if (scKey[0] != 0) {
        if (!hmacMessage(scMessage, scKey, iSalt++, &scHmac, &scSaltedMessage)) {
            printL("Failed to HMAC message");
            return;
        }
        scHmacMessage = new char[32 + 4 + iStrLen];
        memcpy(scHmacMessage, scHmac, 32);
        memcpy(scHmacMessage + 32, scSaltedMessage, 4 + iStrLen);
        scMessage = scHmacMessage;
        iStrLen += 32 + 4;
    }

    mtXML.lock();
    if (socketDesc < 0) ConnectMavLink();
    sendto(socketDesc, scMessage, iStrLen, MSG_CONFIRM, (const struct sockaddr*)&addressMavLink, sizeof(addressMavLink));
    mtXML.unlock();

    if (scKey[0] != 0) {
        OPENSSL_free(scHmac);
        delete[] scSaltedMessage;
        delete[] scHmacMessage;
    }
    #endif
}

char* FlightController::SendRecXML(const char* _scMessage)
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
    char* scSaltedMessage = nullptr;
    char* scHmac = nullptr;
    char* scHmacMessage = nullptr;
    const char* scMessage = _scMessage;
    int iStrLen = strlen(scMessage);
    if (scKey[0] != 0) {
        if (!hmacMessage(scMessage, scKey, iSalt++, &scHmac, &scSaltedMessage)) {
            printL("Failed to HMAC message");
            return nullptr;
        }
        scHmacMessage = new char[32 + 4 + iStrLen];
        memcpy(scHmacMessage, scHmac, 32);
        memcpy(scHmacMessage + 32, scSaltedMessage, 4 + iStrLen);
        scMessage = scHmacMessage;
        iStrLen += 32 + 4;
    }

    mtXML.lock();
    if (socketDesc < 0) ConnectMavLink();
    sendto(socketDesc, scMessage, iStrLen, MSG_CONFIRM, (const struct sockaddr*)&addressMavLink, sizeof(addressMavLink));
    char* scResponse = new char[512];
    char* scBufferStart = scResponse;
    uint32_t* size = (uint32_t*)scResponse; // Mesage length is beginning of message
    int iLength = 0;
    int iCount = 0;
    bool bSuccess = true;
    do {
        iLength = recv(socketDesc, scBufferStart, 511, 0);
        if (iLength < 0) {
            // It failed, give it __msec then try one more time
            std::this_thread::sleep_for(std::chrono::milliseconds(iUdpTimeoutUs / 1000));
            iLength = recv(socketDesc, scBufferStart, 511, 0);
            if (iLength < 0) {
                // Still failed...give up
                bSuccess = false;
                break;
            }
        }
        // If we didn't get 4 byte, we don't have a valid message length, so fail
        if (iLength < 4) {
            bSuccess = false;
            break;
        }
        scBufferStart += iLength;
    } while (iLength < *size && iCount++ < 1);
    if (!bSuccess) {
        delete[] scResponse;
        mtXML.unlock();
        if (scKey[0] != 0) {
            OPENSSL_free(scHmac);
            delete[] scSaltedMessage;
            delete[] scHmacMessage;
        }
        return nullptr;
    }
    
    scResponse[iLength] = 0;
    mtXML.unlock();
    if (scKey[0] != 0) {
        OPENSSL_free(scHmac);
        delete[] scSaltedMessage;
        delete[] scHmacMessage;
    }
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

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = iUdpTimeoutUs;
    setsockopt(socketDesc, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    #endif
}

EVP_PKEY* hmacCreateKey(const char* _scKey)
{
    if (_scKey == nullptr) return nullptr;

    // Create a buffer that's plenty big enough to hold the key, zero it out (memset?)
    char scKey[EVP_MAX_MD_SIZE];
    for (int i = 0; i < EVP_MAX_MD_SIZE; i++) {
        scKey[i] = 0;
    }
    size_t uKeyLength = strlen(_scKey);
    if (uKeyLength > EVP_MAX_MD_SIZE) uKeyLength = EVP_MAX_MD_SIZE;
    strncpy(scKey, _scKey, uKeyLength);
    
    // Get the key type, make sure sizes will work
    EVP_PKEY* pKey = nullptr;
    bool bReturn = true;
    const EVP_MD* pMd = EVP_get_digestbyname("SHA256");
    if (pMd == nullptr) {
        printf("EVP_get_digestbyname failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    int size = 0;
    if (bReturn) {
        size = EVP_MD_size(pMd);
        if(size < 16) {
            printf("EVP_MD_size failed, error 0x%lx\n", ERR_get_error());
            bReturn = false;
        }
    }
    if (bReturn && (size > EVP_MAX_MD_SIZE)) {
       printf("EVP_MD_size is too large\n");
       bReturn = false;
    }

    // Create the key itself
    if (bReturn) {
        pKey = EVP_PKEY_new_mac_key(EVP_PKEY_HMAC, NULL, (const unsigned char*)scKey, size);
        if (pKey == nullptr) {
            printf("EVP_PKEY_new_mac_key failed, error 0x%lx\n", ERR_get_error());
            bReturn = false;
        }
    }
    
    return pKey;
}

bool hmacMessage(std::string strMessage, std::string strKey, int iSalt, char** scHmac, char** scSaltAndMessageOut)
{
    *scHmac = nullptr;
    *scSaltAndMessageOut = nullptr;

    // MessageOut will be salt+message. So we'll allocate strMessage length + 4 bytes (int salt)
    size_t uMessageLength = strMessage.length() + 4;
    char* scSaltAndMessage = new char[uMessageLength];
    // Assign the salt to the beginning of the output message
    *((int32_t*)scSaltAndMessage) = iSalt;
    // Copy the message after where we put the 4 byte salt
    strncpy(scSaltAndMessage + 4, strMessage.c_str(), strMessage.length());
    bool bReturn = true;

    // Create the message digest context
    EVP_MD_CTX* pSslCtx = nullptr;
    pSslCtx = EVP_MD_CTX_create();
    if (pSslCtx == nullptr) {
        printf("EVP_MD_CTX_create failed, error 0x%lx\n", ERR_get_error());
        delete[] scSaltAndMessage;
        return false;
    }
    // Get the message digest for sha256
    const EVP_MD* pMd = EVP_get_digestbyname("SHA256");
    if(pMd == nullptr) {
        printf("EVP_get_digestbyname failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    // Create the hmac/sha256 digest
    if (bReturn && (EVP_DigestInit_ex(pSslCtx, pMd, NULL) != 1)) {
        printf("EVP_DigestInit_ex failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }

    // Get the key for the hmac
    EVP_PKEY* pKey = nullptr;
    if (bReturn) {
        pKey = hmacCreateKey(strKey.c_str());
        if (pKey == nullptr) bReturn = false;
    }
    
    // Attach the key to the digest
    if (bReturn && (EVP_DigestSignInit(pSslCtx, NULL, pMd, NULL, pKey) != 1)) {
        printf("EVP_DigestSignInit failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    
    // Add the message to the digest
    if (bReturn && (EVP_DigestSignUpdate(pSslCtx, scSaltAndMessage, uMessageLength) != 1) ) {
        printf("EVP_DigestSignUpdate failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }

    // Get the size of the digest, allocate enough room for the output
    size_t req = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, NULL, &req) != 1)) {
        printf("EVP_DigestSignFinal failed (1), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    if (bReturn && (req == 0)) {
        printf("EVP_DigestSignFinal failed (2), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    } else {
        *scHmac = (char*)OPENSSL_malloc(req);
    }
    if (bReturn && (*scHmac == nullptr)) {
        printf("OPENSSL_malloc failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }

    // Get the digest and verify size is correct
    size_t slen = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, (unsigned char*)(*scHmac), &slen) != 1)) {
        printf("EVP_DigestSignFinal failed (3), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    if (bReturn && (req != slen)) {
        printf("EVP_DigestSignFinal failed, mismatched signature sizes %ld, %ld", req, slen);
        bReturn = false;
    }

    if (!bReturn) {
        printf("Failed to hmac things\n");
        delete[] scSaltAndMessage;
        if (*scHmac != nullptr) {
            OPENSSL_free(*scHmac);
        }
    } else {
        *scSaltAndMessageOut = scSaltAndMessage;
    }
    if (pSslCtx) EVP_MD_CTX_destroy(pSslCtx); // Renamed EVP_MD_CTX_free in openssl 1.1.0
    EVP_PKEY_free(pKey);

    return bReturn;
}
