#pragma once

#include "navigation.h"
#include "data_types.h"
#include "dataLogger.h"

class FlightController {
public:
    FlightController(DataLogger& logger);

    void SetDirection(float fHeadingOffsetDeg); // Store the currently programmed route, adjust heading
    void SetDestination(float fHeadingOffsetDeg, float fDistance); // Store route, adjust heading, after distance will automatically restore route

    void RestoreRouteNaive();
    void RestoreRouteClean();

private:
    void SaveRoute();
    void runDepth(); // Thread function to continuously send depth info for obstacle avoidance

    DataLogger& logger;
    std::thread* pThreadDepth;
};

