#pragma once

#include <vector>

#include "cameraMulti.h"
#include "comms.h"
#include "locksignal.h"

class Navigator {
public:
    Navigator(float (*activ)(const float&));
    virtual ~Navigator() {}
    virtual void newImageRGB(image imgRGB, bool bSignal) { }
    virtual void newImageIR(image imgIR, bool bSignal) { }
    virtual void newImageDepth(image imgDepth, bool bSignal) { }
    virtual void newImageDepthRaw(cameraFrame_depth depthRaw, bool bSignal) { }
    virtual void newImageREdge(image newImageREdge, bool bSignal) { }

    enum dirty {
        SPEED = 0x01,
        CLIMB = 0x02,
        STEER = 0x04,
        DEST_X = 0x08,
        DEST_Y = 0x10,
        DEST_Z = 0x20
    };
    float getSpeed() { return fSpeed; }
    float getClimb() { return fClimb; }
    float getSteer() { return fSteer; }
    float getConfidenceRaw() { return activ(_getConfidenceRaw()); }
    int getDestX() { return iX; }
    int getDestY() { return iY; }
    int getDestZ() { return iZ; }
    float getConfidenceDest() { return activ(_getConfidenceDest()); }

    dirty isDirty() const { return dirty(flagDirty); }
    void setClean() { flagDirty = 0; }

    void setPriority(float _fPriority) { fPriority = _fPriority; }
    float getPriority() { return fPriority; }

    //void setActivationFunction(activ _activ_func) { active_func = _active_func; }

    virtual void operator()(Signal*) { }

protected:
    virtual float _getConfidenceRaw() { return 0.0; }
    virtual float _getConfidenceDest() { return 0.0; }
    
    int flagDirty;
    float fSpeed;
    float fClimb;
    float fSteer;
    int iX;
    int iY;
    int iZ;

private:
    float fPriority;
    float (*activ)(const float&);
};

class Drive {
public:
    Drive(std::vector<Navigator*>& arNav);
    virtual ~Drive() {}

    virtual bool _decide(Navigator& nav, Navigator::dirty isDirty) = 0;

    float getSpeed() const { return fSpeed; }
    float getSteer() const { return fSteer; }
    float getClimb() const { return fClimb; }

    virtual int getLoopTimeoutMS() const { return 100;}

protected:
    std::vector<Navigator*>& arNav;
    float fSpeed;
    float fSteer;
    float fClimb;
};

class Navigator_SimpleObstruct2D:public Navigator {
public:
    Navigator_SimpleObstruct2D();
    virtual ~Navigator_SimpleObstruct2D();
    
    virtual void newImageRGB(image imgRGB, bool bSignal) override;
    virtual void newImageDepth(image imgDepth, bool bSignal) override;
    virtual void newImageDepthRaw(cameraFrame_depth depthRaw, bool bSignal) override;

    virtual void operator()(Signal*) override;

protected:
    virtual float _getConfidenceRaw() override;

private:
    bool processTerrain();
    bool processObstruct();
    bool processAnimals();

    bool bNewImgRGB;
    bool bNewImgDepth;
    bool bNewImgDepthRaw;
    cameraMulti* pCamera;
    Signal* psigImages;
    image imgRGB;
    image imgDepth;
    cameraFrame_depth *pDepthRaw;
    Mutex mtImages;
};

class Drive_TwinTrack: public Drive {
public:
    Drive_TwinTrack(std::vector<Navigator*>& arNav);
    virtual bool _decide(Navigator& nav, Navigator::dirty isDirty) override;
};

template<class DriveType>
class DroneNavigation {
public:
    DroneNavigation();
    ~DroneNavigation();

    void operator()(void);

    void addNav(Navigator* pNav);

private:
    void _sendGuidance();

    Signal signalMainMessage;
    std::vector<Navigator*> arNav;
    DriveType drive;
};

// ***************
// DroneNavigation

template<class DriveType>
DroneNavigation<DriveType>::DroneNavigation()
: arNav()
, drive(arNav)
{
    printf("DroneNavigation\n");
}

template<class DriveType>
DroneNavigation<DriveType>::~DroneNavigation()
{
    for (int i = 0; i < arNav.size(); i++) {
        delete arNav[i];
    }
}

template<class DriveType>
void DroneNavigation<DriveType>::operator()(void)
{
    printf("DroneNavigation Thread\n");
    std::vector<std::thread*> arNavThreads(arNav.size());
    for (int i = 0; i < arNav.size(); i++) {
        arNavThreads[i] = new std::thread(*(arNav[i]), new Signal(signalMainMessage));
    }

    bool bDone = false;
    do {
        for (int i = 0; i < arNav.size(); i++) {
            Navigator::dirty isDirty = arNav[i]->isDirty();
            if (isDirty) {
                arNav[i]->setClean();
                if (drive._decide(*(arNav[i]), isDirty)) {
                    _sendGuidance();
                }
            }
        }
        signalMainMessage.wait(drive.getLoopTimeoutMS());
    } while (!bDone);
}

template<class DriveType>
void DroneNavigation<DriveType>::addNav(Navigator* pNav)
{
    arNav.push_back(pNav);
}

template<class DriveType>
void DroneNavigation<DriveType>::_sendGuidance()
{
    char strCommand[500];
    snprintf(   strCommand, 
                500,
                "<page><command><speed>%1.5f</speed></command><command><steer>%1.5f</steer></command><command><climb>%1.5f</climb></command></page>",
                drive.getSpeed(),
                drive.getSteer(),
                drive.getClimb());
    if (!sendXML(strCommand)) {
        printf("Failed to send guidance\n");
    }
}
