#pragma once

#include "cameraMulti.h"
#include "data_types.h"
#include "dataLogger.h"
#include "planner.h"

// Place holders with the jetson inference networks...once that's working...
class detectNet;
class segNet;

class ObjectDetection {
public:
    ObjectDetection(DataLogger& logger);

    void SetNewImage(image& img);
    void run();

private:
    bool ProcessImage(image& img);

    DataLogger& logger;
    detectNet* pNetwork;
    ThreadMutex mt;
    Signal sigNewImage;
    StopSignal ss;
    image imgRecent;
};

class TerrainSegmentation {
public:
    TerrainSegmentation(DataLogger& logger);
    ~TerrainSegmentation();

    bool IsHabitatVisible();
    float GetHabitatDirection();
    void SetNewImage(image& img);
    void run();

    enum Classes {
        SKY = 0,
        TREES,
        WATER,
        TERRAIN,
        HABITAT = 15
    };

private:
    bool ProcessImage(image& img);
    bool ScanMask(uint8_t* pMask, int iWidth, int iHeight);

    bool bHabitatVisible;
    float fBestDirection;
    float fImageFOV;
    DataLogger& logger;
    segNet* pNetwork;
    uint8_t* pMask; // only access from thread
    std::vector<const char*> vecClassLabel;
    ThreadMutex mt;
    Signal sigNewImage;
    StopSignal ss;
    image imgRecent;
};

class Navigator {
public:
    Navigator(FlightController& control, DataLogger& logger, CameraController& camera);

    void runNavigator();

private:
    FlightController& control;
    DataLogger& logger;
    CameraController& camera;
    std::thread* pThreadNav;
    std::thread* pThreadDet;
    std::thread* pThreadSeg;
    StopSignal ssMain;
    ObjectDetection detector;
    TerrainSegmentation segmenter;
};

