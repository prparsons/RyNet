#pragma once

#include <thread>
#include <vector>
#include "locksignal.h"
#include "data_types.h"

using namespace std;

class ServerLoop {
public:
    ServerLoop();

    void run(void);

    void SetImage(image& im);

    // Drive data
    void SetMaxDriveValues(float fMaxSteer, float fMaxSpeed, float fMaxClimb);
    void SetSpeed(float fSpeed);
    void SetSteer(float fSteer);
    void SetClimb(float fClimb);
    void SetOrientation(Orientation orient);

    // Drive Planning
    void SetEquation(DestinationEquation eq);

    // Navigator
    void SetObstructions(const std::vector<Obstruction>& arObstructions);

private:
    void updateGPU();

    Mutex mt;
    ThreadLock mtData;
    Signal sigImage;
    // Raw drive
    float fSpeed;
    float fSteer; // -1 <-> 1 full left to full right
    float fClimb; // -1 <-> 1 full dive to full climb
    float fMaxSteer; // Max steering in degrees per sec
    float fMaxSpeed; // m/s
    float fMaxClimb; // m/s
    Orientation orient;
    // Drive planning
    DestinationEquation equation;
    // Navigator
    std::vector<Obstruction> arObstructions;
    //people
    // Machine stats
    int iGPU_Temp;
    int iGPU_Usage;
    int iGPU_RamFree; //shared system memory?
    //cpu temp
    //cpu usage
    //ram usage (free ram)
    // Timing variable
    
};
