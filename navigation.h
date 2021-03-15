#pragma once

#include <vector>
#include "cameraMulti.h"

#define PI 3.141592653590
#define PI_2 1.570796326794
#define PIx2 6.283185307180

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
    DestinationEquation(float fA, float fB, float fC, float fD, float fE, float fF);
    DestinationEquation(float fXSin, float fXCos, float fC, float fYSin, float fYCos, float fF, bool);

    Destination GetLocation(float fT) const;
    void GetDeriv(float& fDx, float& fDy, float fT) const;
    float GetRouteLength() const { return 0; }

    float fT;
    float fT_min;
    float fT_max;

private:
    bool bQuadratic;
    float fA, fB, fC; // x(t)
    float fD, fE, fF; // y(t)
    float fXSin, fXCos; // x(t) = fXSin * sin(t) + fXCos * cos(t) + fC
    float fYSin, fYCos; // y(t) = fYSin * sin(t) + fYCos * cos(t) + fF
};

// Obstructions should always assume current position is at 0,0
class Obstruction {
public:
    Obstruction(float* arDepthBuffer, int iWidth, int iHeight, float fHorizFOV);
    ~Obstruction();

    bool IsObstructed(const DestinationEquation&, float fTStart, float fOffsetX = 0, float fOffsetY = 0); // returns confidence of obstruction 0 - 255
    void setFloor(int _iFloor) { iFloor = _iFloor; }

private:
    float* arObstructed;
    float* arAngles;
    float fHorizFOV;
    int iWidth;
    int iHeight;
    int iFloor;
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

class Navigator_PersonFinder:public Navigator {
public:
    Navigator_PersonFinder();
    virtual ~Navigator_PersonFinder();
    
    virtual void newImageRGB(image imgRGB, bool bSignal) override;

    virtual void _run(Signal* psigMain) override;

protected:
    virtual float _getConfidenceDest() override;

private:
    bool process();

    bool bNewImgRGB;
    cameraMulti* pCamera;
    Signal* psigImages;
    image imgRGB;
    Mutex mtImages;
};

class Navigator_SimpleDepthObstruct:public Navigator {
public:
    Navigator_SimpleDepthObstruct();
    virtual ~Navigator_SimpleDepthObstruct();
    
    virtual void newImageDepth(image imgDepth, bool bSignal) override;
    virtual void newImageDepthRaw(cameraFrame_depth depthRaw, bool bSignal) override;

    virtual void _run(Signal* psigMain) override;

private:
    bool processObstruct();

    bool bNewImgDepth;
    bool bNewImgDepthRaw;
    cameraMulti* pCamera;
    Signal* psigImages;
    image imgDepth;
    cameraFrame_depth *pDepthRaw;
    Mutex mtImages;
};
