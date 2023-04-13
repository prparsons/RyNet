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
    ~ObjectDetection();

    void SetNewImage(image& img);
    void run();
    void SendStopSignal();

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
    void SendStopSignal();
    void SetTargetClass(uint8_t uTarget);

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
    uint8_t uTargetClass;
    float fBestDirection;
    float fImageFOV; // should get this from camera
    DataLogger& logger;
    segNet* pNetwork;
    uint8_t* pMask; // only access from thread (this really doesn't even need to be a class member)
    ThreadMutex mt;
    Signal sigNewImage;
    StopSignal ss;
    image imgRecent;
};

class Navigator:public InterruptStop {
public:
    Navigator(FlightController& control, DataLogger& logger, CameraController& camera);
    ~Navigator();
    
    void Start();
    void Join();
    virtual void SignalStop();
    void Stop();

    void SetTargetClass(uint8_t uTarget) { segmenter.SetTargetClass(uTarget); }

private:
    void runNavigator();

    time_t tLastChange;
    time_t tMinTimeBetweenChanges;
    FlightController& control;
    DataLogger& logger;
    CameraController& camera;
    StopSignal ssMain;
    std::thread* pThread;
    ObjectDetection detector;
    TerrainSegmentation segmenter;
};

