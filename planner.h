#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include "cameraMulti.h"
#include "data_types.h"
#include "dataLogger.h"

/*
    Plan for connecting to drone:
    This connects to a python script and relays commands to it via UDP (i left tcp in there too).
    Python script can use dronekit to connect, saving the work of having to figure out all the
    necessary mavlink commands needed. I think this should also mean we don't have to worry about
    downloading the route to this program...can let the script handle the saving/loading of waypoints.

*/

class CoordNode {
public:
    CoordNode(float fLat, float fLon);
    ~CoordNode();

    float GetDistance(float fLatQuery, float fLonQuery) const;
    const CoordNode* GetClosest(float fLatQuery, float fLonQuery) const;
    const CoordNode* GetClosest(float fLatQuery, float fLonQuery, float& fDistance) const;
    void InsertPoint(float fLatNew, float fLonNew);
    void ResetPosition(float fLat, float fLon); // Clears list and reset position

    void DestroyChildren();
    void Print() const;

private:
    float fLat;
    float fLon;

    CoordNode* nw;
    CoordNode* ne;
    CoordNode* se;
    CoordNode* sw;
};

class Route
{
    float fLat;
    float fLon;
    Route* pNext;
};

class FlightController {
public:
    ~FlightController();

    static FlightController* GetFlightController(DataLogger& logger, CameraController& camera);
    static void DestroyFlightController();

    void WaitForTakeoff(int iAltitude, const bool& bAbort);
    void SetUdpTimout(int iTimeoutUs);
    void SetMissionSpeed(int iMissionSpeedMS);
    void SetExtraDistance(int iExtraGuidedDistance);
    void SetRouteClean(int iClean); // 1 = simple, 2 = 2opt

    void Start(bool bStartDepth, bool bStartPointLogger, bool bStartIr, int iMaxDist);
    void Start(bool bStartDepth, bool bStartPointLogger, bool bStartIr);

    static bool GetPosition(float& fLat, float& fLon);
    static bool GetHeading(float& fHeading);
    bool GetSpeed(float& fSpeedMS); 
    static bool GetAltitude(float& fAltitude);
    bool GetMode(char* scMode, int iStrLen);
    int GetBattery();
    Route* GetRoute(); // Returns the current route. Caller owns object


    // Can we travel in the indicated direction for the indicated time.
    //  0 == No. Positive number is distance direction is valid for.
    int CheckDirectionValid(float fDirection, time_t travelTime);

    void SetDirection(float fHeadingOffsetDeg); // Store the currently programmed route, adjust heading
    void SetDestination(float fHeadingOffsetDeg, float fDistance); // Store route, adjust heading, after distance will automatically restore route
    void SetRTL();
    void SetAltitude(float fAltitudeMeters);
    void RestoreRoute();
    void SetRoute(Route* pNewRoute, bool bSetAuto); // Replaces current route with this one, will takeover ownership.

    void SetServerAddress(const char* scServerAddress);
    void SetUdpKey(const char* scUdpKey);

private:
    FlightController(DataLogger& logger, CameraController& camera);

    void SaveRoute();
    void SendXML(const char* sc);
    char* SendRecXML(const char* sc);
    bool VerifyMessage(const char* _scMessage, uint32_t uExpectedSalt);
    void ConnectMavLink();

    void runDepth(Signal sig, StopSignal ss); // Thread function to continuously send depth info for obstacle avoidance
    void runPointLogger(StopSignal ss); // Thread function to continuously retrieve and store visited points when in guided mode
    void runIr(Signal sig, StopSignal ss);

    DataLogger& logger;
    CameraController& camera;
    std::thread* pThreadDepth;
    std::thread* pThreadPoint;
    std::thread* pThreadIr;
    
    Signal sigCameraDepth;
    Signal sigIr;
    StopSignal ssDepth;
    StopSignal ssPtLog;
    StopSignal ssIr;
    ThreadMutex mtXML;

    struct sockaddr_in addressMavLink;
    int socketDesc;
    int iUdpTimeoutUs;
    int iMaxDist;
    char scMavLinkAddress[32];
    const int iMavlinkPort = 3636;
    char scKey[33];
    int iSalt;
    float fHomeLat;
    float fHomeLon;
    int iMissionSpeedMS;
    int iExtraGuidedDistance;
    int iRouteCleanType;
};

