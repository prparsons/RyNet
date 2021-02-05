#pragma once

#include <vector>
#include "cameraMulti.h"

class Destination {
public:
    Destination();

    bool operator==(const Destination& other);
    bool operator!=(const Destination& other);

    float fX, fY, fZ;
};

class DestinationEquation {
public:
    DestinationEquation();

    Destination GetLocation(float fT) const;

    float fT;

private:
    float fA, fB, fC; // x(t)
    float fD, fE, fF; // y(t)
};

class Obstruction {
public:
    Obstruction(int iWidth, int iHeight);
    ~Obstruction();

    u_int8_t IsObstructed(const DestinationEquation&, float fTStart); // returns confidence of obstruction 0 - 255

private:
    u_int8_t* uObstructed;
};

class Orientation {
public:
    Orientation(): fPitch(0), fRoll(0), fYaw(0) {}

    float fPitch;
    float fRoll;
    float fYaw;
};

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
    float getSpeed() { return fSpeed; } // 0 <-> 1 modifier of current speed
    Orientation getOrientation() { return orient; } //-1 <-> +1 change orientation respective to direction of travel (-1 is look 180 left, 1 is 180 right)
    float getConfidenceRaw() { return activ(_getConfidenceRaw()); }
    Destination getDest() { return dest; }
    float getConfidenceDest() { return activ(_getConfidenceDest()); }
    virtual bool getObstructions(std::vector<Obstruction>&) { return false; }

    dirty isDirty() const { return dirty(flagDirty); }
    void setClean() { flagDirty = 0; }

    void setPriority(float _fPriority) { fPriority = _fPriority; }
    float getPriority() { return fPriority; }

    virtual void run(Signal* psigMain) { _run(psigMain); }
    virtual void _run(Signal* psigMain) = 0;
    StopSignal ss;

protected:
    virtual float _getConfidenceRaw() { return 0.0; }
    virtual float _getConfidenceDest() { return 0.0; }
    
    int flagDirty;
    float fSpeed;
    Orientation orient;
    Destination dest;

private:
    float fPriority;
    float (*activ)(const float&);
};

class Navigator_SimpleObstruct2D:public Navigator {
public:
    Navigator_SimpleObstruct2D();
    virtual ~Navigator_SimpleObstruct2D();
    
    virtual void newImageRGB(image imgRGB, bool bSignal) override;
    virtual void newImageDepth(image imgDepth, bool bSignal) override;
    virtual void newImageDepthRaw(cameraFrame_depth depthRaw, bool bSignal) override;

    virtual void _run(Signal* psigMain) override;

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
