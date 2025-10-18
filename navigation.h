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
    ObjectDetection(DataLoggerWriter* pLogger);
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
    DataLoggerWriter* pLogger;
    detectNet* pNetwork;
    std::thread* pThread;
    ThreadMutex mt;
    Signal sigNewImage;
    StopSignal ssMain;
    CameraFrame frmRecent;
};

class TerrainSegmentation {
public:
    TerrainSegmentation(DataLoggerWriter* pLogger);
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

    DataLoggerWriter* pLogger;
    segNet* pNetwork;
    std::thread* pThread;

    ThreadMutex mt;
    Signal sigNewImage;
    StopSignal ssMain;
    CameraFrame frmRecent;
};

class Navigator:public I_InterruptStop {
public:
    Navigator(FlightController& control, DataLoggerWriter* pLogger, PipeReceiverRGB* pPipeRGB, bool bStartBatteryMonitor);
    ~Navigator();
    
    void Start();
    void Join();
    void SignalStop();

    void SetTargetClass(uint8_t uTarget) { segmenter.SetTargetClass(uTarget); }
    void SetNumberImageSlices(int iNumSlices);
    void SetTimeBetweenChanges(int iTime);
    void SetValidPixels(uint32_t uValidPixels);
    void SetRouteClean(int iClean); // 1 = simple, 2 = 2opt

private:
    Route* RestoreRoute(Route* pRoute);

    void runBattery(StopSignal ss, StopSignal ssMainThread);
    void runNavigator(PipeReceiverRGB& pipeRGB, StopSignal ss);

    // ONLY access from runBattery. runNavigator will access it AFTER it has joined runBattery
    bool bSendRTL;
    bool bStartBatteryMonitor;
    int8_t iRouteCleanType;
    time_t tLastChange;
    time_t tMinTimeBetweenChanges;

    FlightController& control;//todo can replace with pipe?
    Route* pRouteOrig;

    StopSignal ssMain;
    DataLoggerWriter* pLogger;
    PipeReceiverRGB* pPipeRGB;
    std::thread* pThread;
    ObjectDetection detector;
    TerrainSegmentation segmenter;
};

