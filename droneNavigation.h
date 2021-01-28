#pragma once

#include "cameraMulti.h"
#include "locksignal.h"


class Navigator {
public:
    virtual ~Navigator() {}
    virtual void newImageRGB(image imgRGB, bool bSignal) { }
    virtual void newImageIR(image imgIR, bool bSignal) { }
    virtual void newImageDepth(image imgDepth, bool bSignal) { }
    virtual void newImageREdge(image newImageREdge, bool bSignal) { }
    
    virtual float getTerrainDirection() { return 0.0; }
    virtual float getTerrainSpeed() { return 0.0; }
    virtual float getTerrainClimb() { return 0.0; }
    virtual float getObstructDirection() { return 0.0; }
    virtual float getObstructSpeed() { return 0.0; }
    virtual float getObstructClimb() { return 0.0; }

    virtual bool isDirty() const { return false; }
    virtual void setClean() {}

    virtual void operator()(Signal*) {}
};

class Drive {
public:
    Drive(Navigator& nav);
    virtual ~Drive() {}

    virtual bool _decide() = 0;

    float getSpeed() const { return fSpeed; }
    float getSteer() const { return fSteer; }
    float getClimb() const { return fClimb; }

    virtual int getLoopTimeoutMS() const { return 100;}

protected:
    Navigator& nav;
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
    virtual float getObstructDirection() override;
    virtual float getObstructSpeed() override;

    virtual bool isDirty() const override { return bIsDirty; }
    virtual void setClean() override { bIsDirty = false; }

    virtual void operator()(Signal*) override;

private:
    bool processTerrain();
    bool processObstruct();
    bool processAnimals();

    bool bIsDirty;
    bool bNewImgRGB;
    bool bNewImgDepth;
    cameraMulti* pCamera;
    Signal* psigImages;
    image imgRGB;
    image imgDepth;
    Mutex mtImages;
};

class Drive_TwinTrack: public Drive {
public:
    Drive_TwinTrack(Navigator& nav);
    virtual bool _decide() override;
};

class DroneNavigation {
public:
    DroneNavigation(Navigator& nav, Drive& drive);

    void operator()(void);
    
    void _decide();
    void _sendGuidance();

private:
    Signal signalMainMessage;

    Navigator& nav;
    Drive& drive;
};
