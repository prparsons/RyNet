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

    static FlightController& GetFlightController(DataLogger& logger, CameraController& camera);
    static void DestroyFlightController();

    static bool GetPosition(float fLat, float fLon, int iTimeoutMS = 100);
    static bool GetHeading(float fHeading, int iTimeoutMS = 100);

    void SetDirection(float fHeadingOffsetDeg); // Store the currently programmed route, adjust heading
    void SetDestination(float fHeadingOffsetDeg, float fDistance); // Store route, adjust heading, after distance will automatically restore route
    void SetRTL();
    void SetAltitude(float fAltitudeMeters);
    void RestoreRouteNaive();
    void RestoreRouteClean();

private:
    FlightController(DataLogger& logger, CameraController& camera);

    void SaveRoute();
    void SendXML(const char* sc);
    char* SendRecXML(const char* sc, int iTimeoutMS = 100);
    void ConnectMavLink();
    void runDepth(); // Thread function to continuously send depth info for obstacle avoidance

    DataLogger& logger;
    CameraController& camera;
    std::thread* pThreadDepth;
    StopSignal ss;

    struct sockaddr_in addressMavLink;
    int socketDesc;
    const char* scMavLinkAddress = "127.0.0.1";
    const int iMavlinkPort = 3636;
};

