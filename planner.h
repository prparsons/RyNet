#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include "cameraMulti.h"
#include "data_types.h"
#include "dataLogger.h"

class FlightController {
public:
    FlightController(DataLogger& logger, CameraController& camera);
    ~FlightController();

    void SetDirection(float fHeadingOffsetDeg); // Store the currently programmed route, adjust heading
    void SetDestination(float fHeadingOffsetDeg, float fDistance); // Store route, adjust heading, after distance will automatically restore route

    void RestoreRouteNaive();
    void RestoreRouteClean();

private:
    void SaveRoute();
    void SendXML(const char* sc);
    char* SendRecXML(const char* sc);
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

