#pragma once

#include "cameraMulti.h"
#include "locksignal.h"

class Navigator {
public:
    virtual ~Navigator() {}
    virtual bool checkTerrain(image imgRGB) { return false; }
    virtual bool checkTerrain(image imgRGB, image imgIR) { return false; }
    virtual bool checkObstruct(image imgRGB) { return false; }
    virtual bool checkObstruct(image imgRGB, image imgDepth) { return false; }
    virtual bool checkAnimals(image imgRGB) { return false; }
    virtual bool checkAltitude(image imgRGB) { return false; }
    virtual bool checkAltitude(image imgRGB, image imgDepth) { return false; }

    virtual bool isDirty() const { return false; }

    virtual void operator()(Signal) {}
};

class Drive {
public:
    Drive();
    virtual ~Drive() {}

    virtual bool _decide() = 0;

    float getSpeed() const { return fSpeed; }
    float getSteer() const { return fSteer; }
    float getClimb() const { return fClimb; }

    virtual int getLoopTimeoutMS() const { return 100;}

protected:
    float fSpeed;
    float fSteer;
    float fClimb;
};

class Navigator_SimpleObstruct2D:public Navigator {
public:
    Navigator_SimpleObstruct2D();
    virtual ~Navigator_SimpleObstruct2D();
    
    virtual bool checkTerrain(image imgRGB) override;
    virtual bool checkObstruct(image imgRGB, image imgDepth) override;


    virtual bool isDirty() const override { return bIsDirty; }

    virtual void operator()(Signal) override;

private:
    bool bIsDirty;

    cameraMulti* pCamera;
};

class Drive_TwinTrack: public Drive {
public:
    virtual bool _decide() override;
};

class DroneNavigation {
public:
    DroneNavigation(Navigator& nav, Drive& drive);

    void operator()(void);
    
    void _decide();
    void _sendGuidance();

private:
    bool bNeedGuidance;

    Navigator& nav;
    Drive& drive;
};
