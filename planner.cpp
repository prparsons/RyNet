#include "planner.h"

#include <string.h>
#include <chrono>
#include <openssl/evp.h>
#include <openssl/err.h>


EVP_PKEY* hmacCreateKey(std::string strKey);
bool hmacMessage(std::string strMessage, std::string strKey, int iSalt, char** scHmac, char** _scMessage);

FlightController* pInstance = nullptr;

FlightController::FlightController(DataLogger& _logger, CameraController& _camera)
: logger(_logger)
, camera(_camera)
, pThreadDepth(nullptr)
, pThreadPoint(nullptr)
, pThreadIr(nullptr)
, sigCameraDepth(_camera.GetSignalDepth())
, socketDesc(-1)
, iSalt(0)
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

int FlightController::GetBattery()
{
    char* sc = SendRecXML("<battery />");
    if (sc == nullptr) return -1;
    int iBattery = atoi(sc);
    delete sc;
    return iBattery;
}

bool FlightController::CheckDirectionValid(float fDirection, time_t travelTime)
{
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
        sprintf(sc, "<obs dir='r'>%5f</obs>", uResultRight * frameDepth.getMetersPerUnit() * 100);
        SendXML(sc);
        sprintf(sc, "<obs dir='m'>%5f</obs>", uResultMiddle * frameDepth.getMetersPerUnit() * 100);
        SendXML(sc);
        sprintf(sc, "<obs dir='l'>%5f</obs>", uResultLeft * frameDepth.getMetersPerUnit() * 100);
        SendXML(sc);

        delete pFrameDepth;
    }
}

void FlightController::runPointLogger(StopSignal ss)
{
    printL("FlightController::runPointLogger");
    bool bGuided = false;
    std::vector<coord> vecVisitedPoints;
    coord currentPosition;
    char scMode[32];

    while (!ss.shouldStop()) {
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

char* FlightController::SendRecXML(const char* _scMessage, int iTimeoutMS)
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
            //cleanup and return
            printL("Failure!!!");
            return;
        }
        scHmacMessage = new char[32 + strlen(scSaltedMessage)];
        iStrLen = 32 + strlen(scSaltedMessage);
        strncpy(scHmacMessage, scHmac, 32);
        strncpy(scHmacMessage + 32, scSaltedMessage, strlen(scSaltedMessage));
        scMessage = scHmacMessage;
    }

    mtXML.lock();
    if (socketDesc < 0) ConnectMavLink();
    sendto(socketDesc, scMessage, iStrLen, MSG_CONFIRM, (const struct sockaddr*)&addressMavLink, sizeof(addressMavLink));
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
            if (scKey[0] != 0) {
                OPENSSL_free(scHmac);
                delete[] scSaltedMessage;
                delete[] scHmacMessage;
            }
            return nullptr;
        }
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
    #endif
}

EVP_PKEY* hmacCreateKey(std::string strKey)
{
    char scKey[EVP_MAX_MD_SIZE];
    for (int i = 0; i < EVP_MAX_MD_SIZE; i++) {
        scKey[i] = 0;
    }
    strncpy(scKey, strKey.c_str(), strKey.length());
    //**not padding the key with anything
    EVP_PKEY* pKey = nullptr;

    // Get the key type, create the digest
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
        } else {
            printf("Key size%d, md_size%d\n", int(strKey.length()), size);
        }
    }
    if (bReturn && (size > strKey.length())) {
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
    if (!bReturn && pKey != nullptr) {
        printf("Should be key failure, but it's not\n");
    }
    
    return pKey;
}

bool hmacMessage(std::string strMessage, std::string strKey, int iSalt, char** scHmac, char** _scMessage)
{
    *scHmac = nullptr;
    *_scMessage = nullptr;

    size_t uMessageLength = strMessage.length() + 4;
    char* scMessage = new char[uMessageLength];
    *((int32_t*)scMessage) = iSalt;
    strncpy(scMessage + 4, strMessage.c_str(), strMessage.length());
    bool bReturn = true;

    // Create the message digest context
    EVP_MD_CTX* pSslCtx = nullptr;
    pSslCtx = EVP_MD_CTX_create();
    if (pSslCtx == nullptr) {
        printf("EVP_MD_CTX_create failed, error 0x%lx\n", ERR_get_error());
        delete[] scMessage;
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

    EVP_PKEY* pKey = nullptr;
    if (bReturn) {
        pKey = hmacCreateKey(strKey);
        if (pKey == nullptr) bReturn = false;
    }
    printf("key creation done\n");
    // Attach the key to the digest
    if (bReturn && (EVP_DigestSignInit(pSslCtx, NULL, pMd, NULL, pKey) != 1)) {
        printf("EVP_DigestSignInit failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign init done\n");
    // Add the message to the digest
    if (bReturn && (EVP_DigestSignUpdate(pSslCtx, scMessage, uMessageLength) != 1) ) {
        printf("EVP_DigestSignUpdate failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign update done\n");

    // Get the size of the digest, allocate enough room for the output
    size_t req = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, NULL, &req) != 1)) {
        printf("EVP_DigestSignFinal failed (1), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign final done\n");
    if (bReturn && (req == 0)) {
        printf("EVP_DigestSignFinal failed (2), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    } else {
        *scHmac = (char*)OPENSSL_malloc(req);
    }
    if (bReturn && (scHmac == nullptr)) {
        printf("OPENSSL_malloc failed, error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("ssl malloc done\n");

    // Get the digest and verify size is correct
    size_t slen = 0;
    if (bReturn && (EVP_DigestSignFinal(pSslCtx, (unsigned char*)(*scHmac), &slen) != 1)) {
        printf("EVP_DigestSignFinal failed (3), error 0x%lx\n", ERR_get_error());
        bReturn = false;
    }
    printf("digest sign final 2 done\n");
    if (bReturn && (req != slen)) {
        printf("EVP_DigestSignFinal failed, mismatched signature sizes %ld, %ld", req, slen);
        bReturn = false;
    }

    if (!bReturn) {
        printf("Failed to hmac things\n");
        delete scMessage;
        if (pSslCtx) EVP_MD_CTX_destroy(pSslCtx);
    } else {
        printf("hmac successful, cleaning up\n");
        EVP_MD_CTX_destroy(pSslCtx);
        *_scMessage = scMessage;
    }

    return bReturn;
}