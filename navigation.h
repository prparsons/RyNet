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

    void SetNewImage(CameraFrame& frm);
    void Start();
    void SendStopSignal();
    
    void SetNumberImageSlices(int iNumSlices);
    static bool SetNetwork(const char* scNetwork);

private:
    bool ProcessImage(CameraFrame& frm);
    void run(Signal sig, StopSignal ss);

    int iNumSlices;
    DataLogger& logger;
    detectNet* pNetwork;
    std::thread* pThread;
    ThreadMutex mt;
    Signal sigNewImage;
    StopSignal ssMain;
    CameraFrame frmRecent;
};

class TerrainSegmentation {
public:
    TerrainSegmentation(DataLogger& logger);
    ~TerrainSegmentation();

    bool IsHabitatVisible();
    float GetHabitatDirection();
    
    void SetNewImage(CameraFrame& frm);
    void Start();
    void SendStopSignal();

    void SetTargetClass(uint8_t uTarget);
    void SetNumberImageSlices(int iNumSlices);
    void SetNumberPixelsValid(uint32_t uValid);
    static bool SetNetwork(const char* scNetwork);

    enum Classes {
        SKY = 0,
        TREES,
        WATER,
        TERRAIN,
        HABITAT = 15
    };

private:
    bool ProcessImage(CameraFrame& frm);
    bool ScanMask(uint8_t* pMask, int iImageId, uint32_t uWidth, uint32_t uHeight);
    void run(Signal sig, StopSignal ss);

    bool bHabitatVisible;
    uint8_t uTargetClass;
    int iNumSlices;
    float fBestDirection;
    float fImageFOV; // should get this from camera
    uint8_t* pMask; // only access from thread (this really doesn't even need to be a class member)
    uint32_t uMaskWidth;
    uint32_t uMaskHeight;
    uint32_t uValidNumPixels;

    DataLogger& logger;
    segNet* pNetwork;
    std::thread* pThread;

    ThreadMutex mt;
    Signal sigNewImage;
    StopSignal ssMain;
    CameraFrame frmRecent;
};

class Navigator:public InterruptStop {
public:
    Navigator(FlightController& control, DataLogger& logger, CameraController& camera, bool bStartBatteryMonitor);
    ~Navigator();
    
    void Start();
    void Join();
    void SignalStop();

    void SetTargetClass(uint8_t uTarget) { segmenter.SetTargetClass(uTarget); }
    void SetNumberImageSlices(int iNumSlices);
    void SetTimeBetweenChanges(int iTime);
    void SetValidPixels(uint32_t uValidPixels);

private:
    void runBattery(StopSignal ss, StopSignal ssMainThread);
    void runNavigator(Signal sig, StopSignal ss);

    // ONLY access from runBattery. runNavigator will access it AFTER it has joined runBattery
    bool bSendRTL;
    bool bStartBatteryMonitor;
    time_t tLastChange;
    time_t tMinTimeBetweenChanges;

    FlightController& control;
    DataLogger& logger;
    CameraController& camera;

    StopSignal ssMain;
    Signal sigMainRGB;
    std::thread* pThread;
    ObjectDetection detector;
    TerrainSegmentation segmenter;
};

