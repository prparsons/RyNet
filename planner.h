#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include "cameraMulti.h"
#include "data_types.h"
#include "dataLogger.h"
#include "route.h"

/*
    Plan for connecting to drone:
    This connects to a python script and relays commands to it via UDP (i left tcp in there too).
    Python script can use dronekit to connect, saving the work of having to figure out all the
    necessary mavlink commands needed. I think this should also mean we don't have to worry about
    downloading the route to this program...can let the script handle the saving/loading of waypoints.

*/

class FlightController {
public:
    ~FlightController();

    static FlightController* GetFlightController(DataLoggerWriter* pLogger, PipeReceiverDepth* pPipeDepth, PipeReceiver_Image* pPipeIR);
    static void DestroyFlightController();

    void WaitForTakeoff(int iAltitude, const bool& bAbort);
    void SetUdpTimout(int iTimeoutUs);
    void SetMissionSpeed(int iMissionSpeedMS);
    void SetExtraDistance(int iExtraGuidedDistance);

    void Start(bool bStartDepth, bool bStartPointLogger, bool bStartIr, int iMaxDist);
    void Start(bool bStartDepth, bool bStartPointLogger, bool bStartIr);

    static bool GetPosition(float& fLat, float& fLon);
    static bool GetHeading(float& fHeading);
    bool GetSpeed(float& fSpeedMS); 
    static bool GetAltitude(float& fAltitude);
    bool GetMode(char* scMode, int iStrLen);
    int GetBattery();
    Route* GetRoute(); // Returns the current route. Caller owns object
    CoordNode* GetVisitedPoints(); // From point logger thread
    PipeSender_ImageData* GetPipeDetection();
    static bool GetNextWaypoint(int16_t& iNextPoint);

    // Can we travel in the indicated direction for the indicated time.
    //  0 == No. Positive number is distance direction is valid for.
    int CheckDirectionValid(float fDirection, time_t travelTime);

    void SetDirection(float fHeadingOffsetDeg); // Store the currently programmed route, adjust heading
    void SetDestination(float fHeadingOffsetDeg, float fDistance); // Store route, adjust heading, after distance will automatically restore route
    void SetRTL();
    void SetAltitude(float fAltitudeMeters);
    void SetRoute(const Route& NewRoute, bool bSetAuto); // Replaces current route with this one, will takeover ownership.

    void SetServerAddress(const char* scServerAddress);
    void SetUdpKey(const char* scUdpKey);

private:
    FlightController(DataLoggerWriter* pLogger, PipeReceiverDepth* pPipeDepth, PipeReceiver_Image* pPipeIR);

    void SaveRoute();
    void SendXML(const char* sc);
    uint32_t SendRecXML(const char* sc); // Used to send message if you want to be able to get a response
    char* GetUdp(uint32_t uSalt, int iTimeoutMs); // Used to get any response
    bool VerifyMessage(const char* _scMessage, uint32_t uMessageLength);
    void ConnectMavLink();

    void runDepth(PipeReceiverDepth& pipeDepth, Signal ss); // Thread function to continuously send depth info for obstacle avoidance
    void runPointLogger(Signal sigStop); // Thread function to continuously retrieve and store visited points when in guided mode
    void runIr(PipeReceiver_Image& pipeIR, Signal ss);
    void runRecv(Signal ss); // Thread that recv's UDP packet and stores them in a map according to salt
    char* recvUdp(); // Contains the udp code itself for runRecv()

    std::string GpxProlog();
    std::string GpxStartTrackSeg(std::string strSegmentType);
    std::string GpxTrackPoint(float fLat, float fLon, float fElev);
    std::string GpxEndTrackSeg();
    std::string GpxWaypoint(std::string strDesc, float fLat, float fLon);

    DataLoggerWriter* pLogger;
    CoordNode startLocation;
    std::thread* pThreadDepth;
    std::thread* pThreadPoint;
    std::thread* pThreadIr;
    std::thread* pThreadRecv;
    
    PipeReceiverDepth* pPipeDepth;
    PipeReceiver_Image* pPipeIR;
    PipeReceiver_ImageData pipeDetect;
    Signal sigDepthStop;
    Signal sigPtLogStop;
    Signal sigIrStop;
    Signal sigRecvStop;
    ThreadMutex mtXML;
    ThreadMutex mtRecv;
    ThreadMutex mtPoints;

    struct sockaddr_in addressMavLink;
    int socketDesc;
    int iUdpTimeoutUs;
    int iMaxDist;
    char scMavLinkAddress[32];
    const int iMavlinkPort = 3636;
    char scKey[33];
    uint32_t uSalt;
    float fHomeLat;
    float fHomeLon;
    int iMissionSpeedMS;
    int iExtraGuidedDistance;
    std::map<uint32_t, char*> mapRecv;
};

