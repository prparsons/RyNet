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

class FlightController {
public:
    ~FlightController();

    static FlightController* GetFlightController(DataLogger& logger, CameraController& camera);
    static void DestroyFlightController();

    void WaitForTakeoff(int iAltitude);
    void Start(bool bStartDepth, bool bStartPointLogger, bool bStartIr, int iMaxDist);

    static bool GetPosition(float& fLat, float& fLon, int iTimeoutMS = 100);
    static bool GetHeading(float& fHeading, int iTimeoutMS = 100);
    bool GetSpeed(float& fSpeedMS); 
    void GetAltitude(float& fAltitude); // If unsuccessful, will leave iAltitude unchanged
    bool GetMode(char* scMode, int iStrLen);
    int GetBattery();

    // Can we travel in the indicated direction for the indicated time
    bool CheckDirectionValid(float fDirection, time_t travelTime);

    void SetDirection(float fHeadingOffsetDeg); // Store the currently programmed route, adjust heading
    void SetDestination(float fHeadingOffsetDeg, float fDistance); // Store route, adjust heading, after distance will automatically restore route
    void SetRTL();
    void SetAltitude(float fAltitudeMeters);
    void RestoreRouteNaive();
    void RestoreRouteClean();

    void SetServerAddress(const char* scServerAddress);

private:
    FlightController(DataLogger& logger, CameraController& camera);

    void SaveRoute();
    void SendXML(const char* sc);
    char* SendRecXML(const char* sc, int iTimeoutMS = 100);
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
    int iMaxDist;
    char scMavLinkAddress[32];
    const int iMavlinkPort = 3636;
    float fHomeLat;
    float fHomeLon;
};

