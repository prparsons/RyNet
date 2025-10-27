#include "planner.h"

#include <string.h>
#include <chrono>
#include <fstream>
#include <openssl/evp.h>
#include <openssl/err.h>

// Messages: uint32_t(size of mesage)32_bytes(hmac digest)uint32_t(salt)
#define RECV_OFFSET (4 + 32 + 4)


EVP_PKEY* hmacCreateKey(const char* _scKey);
bool hmacMessage(std::string strMessage, std::string strKey, int iSalt, char** scHmac, char** scSaltAndMessageOut);

FlightController* pInstance = nullptr;

FlightController::FlightController(DataLoggerWriter* _pLogger, PipeReceiverDepth* _pPipeDepth, PipeReceiver_Image* _pPipeIR)
: pLogger(_pLogger)
, startLocation(0,0)
, pThreadDepth(nullptr)
, pThreadPoint(nullptr)
, pThreadIr(nullptr)
, pPipeDepth(_pPipeDepth)
, pPipeIR(_pPipeIR)
, pipeDetect(16, true)
, pipeSegment(16, true)
, socketDesc(-1)
, uSalt(1)
, iUdpTimeoutUs(100000)
, iMaxDist(-1)
, fHomeLat(0)
, fHomeLon(0)
, iMissionSpeedMS(8)
, iExtraGuidedDistance(10)
{
    scKey[0] = 0;
    strcpy(scMavLinkAddress, "127.0.0.1");
    memset(&addressMavLink, 0, sizeof(addressMavLink));
}

FlightController* FlightController::GetFlightController(DataLoggerWriter* pLogger, PipeReceiverDepth* pPipeDepth, PipeReceiver_Image* pPipeIR)
{
    if (pInstance == nullptr) {
        pInstance = new FlightController(pLogger, pPipeDepth, pPipeIR);
        return pInstance;
    }
    return nullptr;
}

void FlightController::DestroyFlightController()
{
    if (pInstance == nullptr) return;

    if (pInstance->pThreadDepth != nullptr) {
        printL("FlightController::DestroyFlightController Stopping depths");
        pInstance->sigDepthStop.signal();
    }
    if (pInstance->pThreadPoint != nullptr) {
        printL("FlightController::DestroyFlightController Stopping point logger");
        pInstance->sigPtLogStop.signal();
    }
    if (pInstance->pThreadIr != nullptr) {
        printL("FlightController::DestroyFlightController Stopping thermal camera");
        pInstance->sigIrStop.signal();
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
    pThreadRecv = new std::thread(&FlightController::runRecv, this, sigRecvStop);
    if (bStartDepth) {
        pThreadDepth = new std::thread(&FlightController::runDepth, this, std::ref(*pPipeDepth), sigDepthStop);
    }
    if (bStartPointLogger) {
        pThreadPoint = new std::thread(&FlightController::runPointLogger, this, sigPtLogStop);
    }
    if (bStartIr) {
        pThreadIr = new std::thread(&FlightController::runIr, this, std::ref(*pPipeIR), sigIrStop);
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
                    pLogger->LogString("Has entered auto");
                }
                GetAltitude(fCurrentAltitude);
            }
        }
    }
    printf("Has reached startup altitude %.3f", fCurrentAltitude);
    pLogger->LogString("Has reached startup altitude");

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

FlightController::~FlightController()
{
    printL("FlightController::~FlightController");
    sigRecvStop.signal();

    if (pThreadRecv->joinable()) pThreadRecv->join();
    delete pThreadRecv;

    if (pThreadDepth != nullptr) {
        sigDepthStop.signal();
        if (pThreadDepth->joinable()) pThreadDepth->join();
        delete pThreadDepth;
    }
    if (pThreadPoint != nullptr) {
        sigPtLogStop.signal();
        if (pThreadPoint->joinable()) pThreadPoint->join();
        delete pThreadPoint;
    }
    if (pThreadIr != nullptr) {
        sigIrStop.signal();
        if (pThreadIr->joinable()) pThreadIr->join();
        delete pThreadIr;
    }
    if (pPipeDepth != nullptr) {
        delete pPipeDepth;
    }
    if (pPipeIR != nullptr) {
        delete pPipeIR;
    }

    // Clear out any unwanted, unclaimed or orphaned messages.
    for (std::map<uint32_t, char*>::iterator iter(mapRecv.begin()); iter != mapRecv.end(); iter++)
    {
        delete[] iter->second;
    }

    if (pLogger != nullptr) {
        delete pLogger;
    }

    //socket
}

bool FlightController::GetPosition(float& fLat, float& fLon)
{
    if (pInstance == nullptr) return false;
    uint32_t uSalt = pInstance->SendRecXML("<pos_retrieve />");
    char* sc = pInstance->GetUdp(uSalt, 100);
    if (sc == nullptr) {
        printL(" GetPosition fail");
        return false;
    }

    // Advance 4 bytes for size, 32 for hmac
    char* scNum = sc + RECV_OFFSET;
    int i;
    for (i = 0; i < strlen(scNum); i++) {
        if (scNum[i] == '\n') break;
    }
    scNum[i] = 0;
    fLat = float(atof(scNum));
    fLon = float(atof(scNum + i + 1));
    delete[] sc;
    return true;
}

bool FlightController::GetHeading(float& fHeading)
{
    if (pInstance == nullptr) return false;
    uint32_t uSalt = pInstance->SendRecXML("<heading_retrieve />");
    char* sc = pInstance->GetUdp(uSalt, 100);
    if (sc == nullptr) {
        printL(" GetHeading fail");
        return false;
    }
    fHeading = atof(sc + RECV_OFFSET);
    delete[] sc;
    return true;
}

bool FlightController::GetSpeed(float& fSpeedMS)
{
    if (pInstance == nullptr) return false;
    uint32_t uSalt = pInstance->SendRecXML("<speed_retrieve />");
    char* sc = pInstance->GetUdp(uSalt, 100);
    if (sc == nullptr) {
        printL(" GetSpeed fail");
        return false;
    }
    fSpeedMS = atof(sc + RECV_OFFSET);
    delete[] sc;
    return true;
}

bool FlightController::GetAltitude(float& fAltitude)
{
    if (pInstance == nullptr) return false;
    uint32_t uSalt = pInstance->SendRecXML("<altitude_retrieve />");
    char* sc = pInstance->GetUdp(uSalt, 100);
    if (sc == nullptr) {
        printL(" GetAltitude fail");
        return false;
    }
    fAltitude = atof(sc + RECV_OFFSET);
    delete[] sc;
    return true;
}

bool FlightController::GetMode(char* scMode, int iStrLen)
{
    if (pInstance == nullptr) return false;
    uint32_t uSalt = pInstance->SendRecXML("<mode_retrieve />");
    char* sc = pInstance->GetUdp(uSalt, 100);
    if (sc == nullptr) {
        printL(" GetMode fail");
        return false;
    }
    strncpy(scMode, sc + RECV_OFFSET, iStrLen);
    delete[] sc;
    return true;
}

int FlightController::GetBattery()
{
    if (pInstance == nullptr) return false;
    uint32_t uSalt = pInstance->SendRecXML("<battery_retrieve />");
    char* sc = pInstance->GetUdp(uSalt, 100);
    if (sc == nullptr) {
        printL(" GetBattery fail");
        return false;
    }
    int iBattery = atoi(sc + RECV_OFFSET);
    delete[] sc;
    return iBattery;
}

Route* FlightController::GetRoute()
{
    if (pInstance == nullptr) return nullptr;
    uint32_t uSalt = pInstance->SendRecXML("<route_retrieve />");
    char* sc = pInstance->GetUdp(uSalt, 5000);
    if (sc == nullptr) {
        printL("GetRoute failed");
        return nullptr;
    }
    char* scRoute = sc + RECV_OFFSET;
    
    uint32_t& uLen = *((uint32_t*)sc);
    Route* pRoute = Route::CreateRoute(scRoute, uLen - RECV_OFFSET);

    delete[] sc;
    return pRoute;
}

CoordNode* FlightController::GetVisitedPoints()
{
    mtPoints.lock();
    CoordNode* pStolen = new CoordNode(startLocation, 0, 0);
    mtPoints.unlock();
    return pStolen;
}

PipeSender_ImageData* FlightController::GetPipeDetection()
{
    return pipeDetect.CreateSender();
}

PipeSender_SegmentData* FlightController::GetPipeSegmentation()
{
    return pipeSegment.CreateSender();
}

bool FlightController::GetNextWaypoint(int16_t& iNextPoint)
{
    if (pInstance == nullptr) return false;
    uint32_t uSalt = pInstance->SendRecXML("<waypoint_current_retrieve />");
    char* sc = pInstance->GetUdp(uSalt, 100);
    if (sc == nullptr) {
        printL(" GetNextWaypoint fail");
        return false;
    }
    iNextPoint = int16_t(atoi(sc + RECV_OFFSET));
    return true;
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

void FlightController::SetRoute(const Route& NewRoute, bool bSetAuto)
{
    std::string strRoute = "<route_set ";
    if (bSetAuto) {
        strRoute += "setauto='true'>";
    } else {
        strRoute += ">";
    }
    strRoute += NewRoute.toString();
    strRoute += "</route_set>";

    SendXML(strRoute.c_str());
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

void FlightController::runDepth(PipeReceiverDepth& pipeDepth, Signal sigStop)
{
    printL("FlightController::runDepth");

    // https://developer.arm.com/architectures/instruction-sets/intrinsics/
    const uint16_t uMax = 65535;
    const uint16_t uUnderflow = 1;
    const uint16x8_t u8Underflow = vld1q_dup_u16(&uUnderflow);

    while (!sigStop.isSignaled()) {
        std::shared_ptr<cameraFrame_depth> pFrameDepth;
        if (!pipeDepth.RetrieveData(pFrameDepth, 1000)) {
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

        // Limit distance messages to 20hz
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void FlightController::runPointLogger(Signal sigStop)
{
    printL("FlightController::runPointLogger");
    bool bGuided = false;
    char scMode[32];

    float fLat = 0;
    float fLon = 0;
    float fAlt = 0;
    GetPosition(fLat, fLon);
    GetAltitude(fAlt);
    // Strings used for storing all the parts of the gpx/kml files
    std::string strTrackGpx;
    std::string strDetectionGpx;
    std::string strPathDocKml;
    std::string strPathPtsKml;
    std::string strDetectionKml; // Waypoints of detections
    std::string strSegmentationKml; // Overlays of habitat
    // File names
    char scFileNameGpx[128];
    time_t timeNow = time(nullptr);
    tm* pTimeStruct = localtime(&timeNow);
    snprintf(scFileNameGpx, 
            127, 
            "run_%d_%d_%d_%d_%d.gpx", 
            pTimeStruct->tm_year + 1900, 
            pTimeStruct->tm_mon + 1,
            pTimeStruct->tm_mday,
            pTimeStruct->tm_hour,
            pTimeStruct->tm_min);
    printf(" Starting GPX file: %s\n", scFileNameGpx);
    {
        std::fstream fileGpx;
        fileGpx.open(scFileNameGpx, std::ios::out | std::ios::app);
        if (fileGpx) {
            fileGpx << GpxProlog() << GpxStartTrackSeg("AUTO");
            fileGpx.close();
        } else {
            printL(" failed to save GPX");//todo throw exception?, bail the whole thing? crash the drone?
        }
    }
    char scFileNameKml[128];
    snprintf(scFileNameKml, 
        127, 
        "run_%d_%d_%d_%d_%d.kml", 
        pTimeStruct->tm_year + 1900, 
        pTimeStruct->tm_mon + 1,
        pTimeStruct->tm_mday,
        pTimeStruct->tm_hour,
        pTimeStruct->tm_min);
    printf(" Starting KML file: %s\n", scFileNameKml);
    {
        std::fstream fileKml;
        fileKml.open(scFileNameKml, std::ios::out | std::ios::app);
        if (fileKml) {
            fileKml << KmlProlog();
            fileKml.close();
        } else {
            printL(" failed to save KML");//todo throw exception?, bail the whole thing? crash the drone?
        }
    }
    strPathDocKml = KmlStartPath("Start of drone flight - AUTO", KmlPathStyle::AUTO);

    // Main loop
    while (!sigStop.isSignaled()) {
        // First part we look to see which mode we're in (guided or auto) and save those points
        // both for gpx/kml files and for the path optimization to be able to delete points
        // from the route that were already visited in guided mode.
        if (bGuided) { // In GUIDED
            bool bValidPos = false;
            // Get the current position, add it to the list of visited points
            if (GetPosition(fLat, fLon)) {
                bValidPos = true;
                GetAltitude(fAlt);

                strTrackGpx += GpxTrackPoint(fLat, fLon, fAlt);
                std::pair<std::string, std::string> kmlStrings = KmlPathPoint(fLat, fLon, fAlt);
                strPathDocKml += kmlStrings.first;
                strPathPtsKml += kmlStrings.second;

                mtPoints.lock();
                startLocation.InsertPoint(fLat, fLon);
                mtPoints.unlock();
            }
            // See if we're in guided or auto
            if (GetMode(scMode, 32)) {
                bGuided = strcmp(scMode, "GUIDED") == 0;
            }
            if (!bGuided) {
                // Just switched to AUTO from GUIDED, so clear visited points
                strTrackGpx += GpxEndTrackSeg();
                strTrackGpx += GpxStartTrackSeg("AUTO");
                strPathDocKml += KmlEndPath(strPathPtsKml);
                strPathDocKml += KmlStartPath("Drone Path - AUTO", KmlPathStyle::AUTO);
                strPathPtsKml = "";

                mtPoints.lock();
                startLocation.ResetPosition(fLat, fLon);
                mtPoints.unlock();
            }
        } else { // in AUTO            
            // See if we're in guided or auto (switched?)
            bool bValidPos = GetPosition(fLat, fLon);
            GetAltitude(fAlt);
            if (GetMode(scMode, 32)) {
                bGuided = strcmp(scMode, "GUIDED") == 0;
                if (bGuided) {
                    // Just switched to GUIDED from AUTO
                    strTrackGpx += GpxEndTrackSeg();
                    strTrackGpx += GpxStartTrackSeg("GUIDED");
                    strPathDocKml += KmlEndPath(strPathPtsKml);
                    strPathDocKml += KmlStartPath("Drone Path - GUIDED", KmlPathStyle::GUIDED);
                    strPathPtsKml = "";

                    mtPoints.lock();
                    startLocation.ResetPosition(fLat, fLon);//todo did we do this if bad point before?
                    mtPoints.unlock();
                }
            }
            if (bValidPos) {
                strTrackGpx += GpxTrackPoint(fLat, fLon, fAlt);
                std::pair<std::string, std::string> kmlStrings = KmlPathPoint(fLat, fLon, fAlt);
                strPathDocKml += kmlStrings.first;
                strPathPtsKml += kmlStrings.second;
            }
        }

        // Now look to see if the detection or segmentation threads have added anything to the respective pipes for the
        // gpx and kml files.
        if (pipeDetect.DataReady()) {
            std::shared_ptr<ImageLogData> pWaypoint;
            while (pipeDetect.RetrieveData(pWaypoint, 0)) {
                strDetectionGpx += GpxWaypoint(pWaypoint->GetString(), pWaypoint->GetLat(), pWaypoint->GetLon());
                strDetectionKml += KmlWaypoint("Detection", pWaypoint->GetString(), pWaypoint->GetLat(), pWaypoint->GetLon(), false);
            }
        }
        if (pipeSegment.DataReady()) {
            std::shared_ptr<std::pair<ImageLogData, std::string>> pWaypoint;
            while (pipeSegment.RetrieveData(pWaypoint, 0)) {
                strSegmentationKml += KmlHabitatOverlay(pWaypoint->first, pWaypoint->second);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Thread is done, now save off the gpx and kml files
    printL(" Saving GPX file");
    {
        std::fstream fileGpx;
        fileGpx.open(scFileNameGpx, std::ios::out | std::ios::app);
        if (fileGpx) {
            fileGpx << strTrackGpx << GpxEndTrackSeg() << strDetectionGpx << "</gpx>";
            fileGpx.close();
        } else {
            printL(" failed to save GPX");//todo throw exception?, bail the whole thing? crash the drone?
        }
    }
    printL(" Saving KML file");
    {
        std::fstream fileKml;
        fileKml.open(scFileNameKml, std::ios::out | std::ios::app);
        if (fileKml) {
            fileKml << strPathDocKml << KmlEndPath(strPathPtsKml) << strDetectionKml << strSegmentationKml;
            //todo in a document << strDetectionKml;//segmentation
            fileKml << "</Folder></kml>";
            fileKml.close();
        } else {
            printL(" failed to save KML");//todo throw exception?, bail the whole thing? crash the drone?
        }
    }
}

void FlightController::runIr(PipeReceiver_Image& pipeIR, Signal sigStop)
{
    printL("FlightController::runIr");

    while (!sigStop.isSignaled()) {
        std::shared_ptr<CameraFrame> pNewImage;
        // Get new image and process it
        if (!pipeIR.RetrieveData(pNewImage, 1000)) {
            continue;
        }
        if (!pNewImage) continue;
        pLogger->LogImageIr(*pNewImage);
    }
}

void FlightController::runRecv(Signal sigStop)
{
    printL("FlightController::runRecv");

    char* buffer = nullptr;
    uint32_t* pSalt = nullptr;

    while (!sigStop.isSignaled()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Get a valid frame (size right, unverified)
        // Frame layout: { uint32_t size, char[32] digest, uint32_t salt, char[] message}
        buffer = recvUdp();
        if (buffer) {
            // Verify that the message is valid (start of digest, size - the size/digest/salt)
            if (!VerifyMessage(buffer + 4, *((uint32_t*)buffer) - 4 - 32 - 4)) {
                printL("Verify Fail");
                delete[] buffer;
                continue;
            }
            // Set up salt variable at right point in message
            pSalt = (uint32_t*)(buffer + 4 + 32);
            // Then add it to the receive map
            mtRecv.lock();
            mapRecv[*pSalt] = buffer;
            mtRecv.unlock();
            // Whatever gets it will clean it up
            buffer = nullptr;
        }
    }
}

char* FlightController::recvUdp()
{
    mtXML.lock();
    char* scResponse = new char[16384];
    char* scBufferStart = scResponse;
    uint32_t* size = (uint32_t*)scResponse; // Mesage length is beginning of message
    *size = 0;
    int iLength = 0;
    int iCount = 0;
    bool bSuccess = true;
    size_t recvSize = 16383;

    if (socketDesc < 0) { // Not initialized yet, let send do it
        mtXML.unlock();
        return nullptr;
    }
    do {
        iLength = recv(socketDesc, scBufferStart, recvSize, 0);

        // If we didn't get 4 byte, we don't have a valid message length, so fail
        if (iLength < 4 + 32 + 4) {
            bSuccess = false;
        } else {
            bSuccess = true;
        }
        // Don't cause problems with a too big "size" for the buffer
        if (*size > recvSize) {
            bSuccess = false;
            break;
        }
        scBufferStart += iLength;
    } while (iLength < *size && iCount++ < 1); // end of the loop that's trying to get "an entire message"

    if (!bSuccess) {
        delete[] scResponse;
        mtXML.unlock();
        return nullptr;
    }
    // Haven't been seeing this yet, so leaving it in here
    if (iLength != *size) {
        printf("Size mismatch: %d  %d\n", iLength, int(*size));
    }
    
    // Set null character at end of the string
    scResponse[*size] = 0;
    mtXML.unlock();

    return scResponse;
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
    mtXML.lock();
    uint32_t usedSalt = uSalt++;
    mtXML.unlock();

    if (scKey[0] != 0) {
        if (!hmacMessage(scMessage, scKey, usedSalt, &scHmac, &scSaltedMessage)) {
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

bool FlightController::VerifyMessage(const char* _scMessage, uint32_t uMessageLength)
{
    const char* scHmacOrig = _scMessage;
    const char* scMessage = _scMessage + 32 + 4; // _scMessage is at start of digest...skip that and the salt for just the message
    const char* scMessageAndSalt = _scMessage + 32;
    bool bReturn = true;

    // Create the message digest context
    EVP_MD_CTX* pSslCtx = nullptr;
    pSslCtx = EVP_MD_CTX_create();
    if (pSslCtx == nullptr) {
        printf("EVP_MD_CTX_create failed, error 0x%lx\n", ERR_get_error());
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
        pKey = hmacCreateKey(scKey);
        if (pKey == nullptr) bReturn = false;
    }
    
    // Attach the key to the digest
    if (bReturn && (EVP_DigestSignInit(pSslCtx, NULL, pMd, NULL, pKey) != 1)) {
        printf("EVP_DigestSignInit failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    
    // Add the message to the digest
    if (bReturn && (EVP_DigestSignUpdate(pSslCtx, scMessageAndSalt, uMessageLength + 4) != 1) ) {
        printf("EVP_DigestSignUpdate failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }

    // Get the size of the digest, allocate enough room for the output
    size_t req = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, NULL, &req) != 1)) {
        printf("EVP_DigestSignFinal failed (1), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    char* scHmac = nullptr;
    if (bReturn && (req == 0)) {
        printf("EVP_DigestSignFinal failed (2), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    } else {
        scHmac = (char*)OPENSSL_malloc(req);
    }
    if (!scHmac) {
        printf("OPENSSL_malloc(%d) failed, error 0x%lx\n", int(req), ERR_get_error());
        bReturn = false;
    }

    // Get the digest and verify size is correct
    size_t slen = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, (unsigned char*)(scHmac), &slen) != 1)) {
        printf("EVP_DigestSignFinal failed (3), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    if (bReturn && (req != slen)) {
        printf("EVP_DigestSignFinal failed, mismatched signature sizes %ld, %ld", req, slen);
        bReturn = false;
    }

    if (pSslCtx) EVP_MD_CTX_destroy(pSslCtx); // Renamed EVP_MD_CTX_free in openssl 1.1.0
    EVP_PKEY_free(pKey);
    // Now do a quick check to make sure the digest matches (can make this faster by comparing bigger things)
    if (bReturn) {
        for (int i = 0; i < slen; i++) {
            if (scHmac[i] != scHmacOrig[i]) {
                bReturn = false;
                printf(" Digest mismatch[Ours]: ");
                for (int i = 0; i < slen; i++) {
                    printf("%02x", scHmac[i]);
                }
                printf("\n Digest mismatch[Theirs]: ");
                for (int i = 0; i < slen; i++) {
                    printf("%02x", scHmacOrig[i]);
                }
                printf("\n");
                break;
            }
        }
    }

    if (scHmac != nullptr) OPENSSL_free(scHmac);
    return bReturn;
}

uint32_t FlightController::SendRecXML(const char* _scMessage)
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
    // Grab the salt and save it (threads, we'll be using it to verify too)
    mtXML.lock();
    uint32_t usedSalt = uSalt++;
    mtXML.unlock();

    // Create the message to send
    if (scKey[0] != 0) {
        if (!hmacMessage(scMessage, scKey, usedSalt, &scHmac, &scSaltedMessage)) {
            printL("Failed to HMAC message");
            return 0;
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
    #endif
    return usedSalt;
}

char* FlightController::GetUdp(uint32_t uSalt, int iTimeoutMs)
{
    bool bFound = false;    
    char* buffer = nullptr;
    do {
        mtRecv.lock();
        // .at() will throw exception if it doesn't exist, so try and get it
        try{
            buffer = mapRecv.at(uSalt);
        } catch (std::out_of_range e)
        {
            // Only thing to really do here
            buffer = nullptr;
        }
        if (buffer != nullptr) {
            // If we got a buffer, clear it from the list and exit the loop
            mapRecv.erase(uSalt);
            mtRecv.unlock();
            bFound = true;
        } else {
            // Didn't find anything, so unlock mutex and check timeout. If time left, then wait.
            mtRecv.unlock();
            if (iTimeoutMs <= 0) return nullptr;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            iTimeoutMs -= 10;
        }
    } while (!bFound);
    // Return whatever was found(nullptr for timeout)
    return buffer;
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

std::string FlightController::GpxProlog()
{
    time_t timeNow = time(nullptr);
    std::tm* timeStruct = gmtime(&timeNow);
    std::stringstream ss;
    ss << std::put_time(timeStruct, "%F") << "T" << std::put_time(timeStruct, "%T") << "Z";

    std::string str = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"\
                       "<gpx version=\"1.1\" creator=\"RyNet\">\n"\
                            "<metadata>\n"\
                                "<name>Flown Route</name>\n"\
                                "<time>";
    str += ss.str();
    str +=                      "</time>\n"\
                            "</metadata>\n";
    return str;
}

std::string FlightController::GpxStartTrackSeg(std::string strSegmentType)
{
    return "<trkseg>\n"\
                "<extensions>\n"
                    "<mode>" + strSegmentType + "</mode>\n"\
                "</extensions>\n";
}

std::string FlightController::GpxTrackPoint(float fLat, float fLon, float fElev)
{
    std::string strGpx = "<trkpt lat=\"";
    strGpx += fLat;
    strGpx += "\" lon=\"";
    strGpx += fLon;
    strGpx += "\"><ele>";
    strGpx += fElev;
    strGpx += "</ele><time>";

    time_t timeNow = time(nullptr);
    std::tm* timeStruct = gmtime(&timeNow);
    std::stringstream ss;
    ss << std::put_time(timeStruct, "%F") << "T" << std::put_time(timeStruct, "%T") << "Z";

    strGpx += ss.str();
    strGpx += "</time></trkpt>\n";
}

std::string FlightController::GpxEndTrackSeg()
{
    return "</trkseg>\n";
}

std::string FlightController::GpxWaypoint(std::string strDesc, float fLat, float fLon)
{
    std::string strGpx = "<wpt lat=\"";
    strGpx += fLat;
    strGpx += "\" lon=\"";
    strGpx += fLon;
    strGpx += "\"><desc>" + strDesc + "</desc></wpt>\n";
    return strGpx;
}

std::string FlightController::KmlProlog()
{
    return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"\
            "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n"\
                "<Folder>\n"\
                    "<name>RyNet_FlownRoute.kml</name>\n"\
                    "<description>\n"\
                        "Contains path flown, waypoints of positive detections, and overlay of habitat.\n"\
                    "</description>\n";
}

std::string FlightController::KmlStartPath(std::string strDesc, KmlPathStyle styleSegment)
{
    //todo unique name
    std::string strPath;
    strPath += "<Document>\n"\
                    "<name>Drone Path</name>\n"\
                    "<description>";
    strPath += strDesc + "</description>\n";
    strPath +=      "<Style id=\"yellowLineGreenPoly\">\n"\
                        "<LineStyle>\n"\
                            "<color>7f00ffff</color>\n"\
                            "<width>4</width>\n"\
                        "</LineStyle>\n"\
                        "<PolyStyle>\n"\
                            "<color>7f00ff00</color>\n"\
                        "</PolyStyle>\n"\
                    "</Style>\n";
    strPath += "<Placemark>\n"\
                    "<name>Path</name>\n"\
                    "<description>List of points drone traveled</description>\n";
    strPath +=      "<styleUrl>#yellowLineGreenPoly</styleUrl>\n"\
                    "<LineString>\n"\
                        "<extrude>1</extrude>\n"\
                        "<tessellate>1</tessellate>\n"\
                        "<altitudeMode>absolute</altitudeMode>\n"\
                        "<coordinates>";
    return strPath;
}

std::pair<std::string, std::string> FlightController::KmlPathPoint(float fLat, float fLon, float fElev)
{
    std::string strCoord;
    strCoord += fLon;
    strCoord += ",";
    strCoord += fLat;
    strCoord += ",";
    strCoord += fElev;

    std::string strWpt = KmlWaypoint("Drone Path point", "", fLat, fLon, true);

    return std::make_pair(strCoord, strWpt);
}

std::string FlightController::KmlEndPath(std::string strPathPoints)
{
    std::string strPathDoc;
    strPathDoc += "</coordinates>\n"\
                "</LineString>\n"\
            "</Placemark>\n";
    strPathDoc += strPathPoints + "</Document>";
    return strPathDoc;
}

//todo<GroundOverlay>

std::string FlightController::KmlWaypoint(std::string strName, std::string strDesc, float fLat, float fLon, bool bTime)
{
    std::string strWpt;
    strWpt += "<Placemark>\n"\
                "<name>";
    strWpt += strName + "</name>\n";
    strWpt +=   "<description>";
    strWpt += strDesc + "</description>\n";
    //todo<altitudeMode>
    if (bTime) {
        time_t timeNow = time(nullptr);
        std::tm* timeStruct = gmtime(&timeNow);
        std::stringstream ss;
        ss << std::put_time(timeStruct, "%F") << "T" << std::put_time(timeStruct, "%T") << "Z";
        strWpt += "<TimeStamp><when>";
        strWpt += ss.str();
        strWpt += "</when></TimeStamp>\n";
    }
    strWpt +=   "<Point>\n"\
                    "<coordinates>";
    strWpt += fLon;
    strWpt += ",";
    strWpt += fLat;
    //todo,altitude</coordinates>
    strWpt +=       "</coordinates>\n"\
                "</Point>\n"\
            "</Placemark>\n";

    return strWpt;
}

std::string FlightController::KmlHabitatOverlay(const ImageLogData& data, std::string strFilename)
{
    std::string strHab;
    strHab += "<GroundOverlay>\n"\
                "<name>";
    strHab += "Habitat Detection</name>\n<description>";
    //todo masks should have a way to be associated with the image
    strHab += "Overlay of the image that triggered the habitat detection.</description>\n<Icon><href>images/";
    strHab += strFilename;
    strHab +=   "</href></Icon>\n"\
                "<LatLonBox>\n"\
                    "<north>";
    strHab += (data.GetLat() + 0.0005);
    strHab += "</north>\n<south>";
    strHab += (data.GetLat() - 0.0005);
    strHab += "</south>\n<east>";
    strHab += (data.GetLon() - 0.0005);
    strHab += "</east>\n<west>";
    strHab += (data.GetLon() + 0.0005);
    strHab +=       "</west>\n"\
                    "<rotation>";
    strHab += data.GetHeading();
    strHab +=       "</rotation>\n"\
                "</LatLonBox>\n"\
            "</GroundOverlay>";

    return strHab;
}
