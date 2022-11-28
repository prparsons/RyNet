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
        HABITAT = 8 //sunrgb, door is 8
    };

private:
    bool ProcessImage(image& img);
    bool ScanMask(uchar3* imgMask, int iWidth, int iHeight);

    bool bHabitatVisible;
    DataLogger& logger;
    segNet* pNetwork;
    float fBestDirection;
    float fImageFOV;
    ThreadMutex mt;
    Signal sigNewImage;
    StopSignal ss;
    image imgRecent;
    uchar3* u3Mask; // only access from thread
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

