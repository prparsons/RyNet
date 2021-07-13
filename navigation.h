#pragma once

#include <vector>
#include "cameraMulti.h"
#include "data_types.h"

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

class detectNet;

class Navigator_PersonFinder:public Navigator {
public:
    Navigator_PersonFinder();
    virtual ~Navigator_PersonFinder();
    
    virtual void newImageRGB(image imgRGB, bool bSignal) override;
    virtual void newImageDepthRaw(cameraFrame_depth depthRaw, bool bSignal) override;

    virtual void _run(Signal* psigMain) override;

protected:
    virtual float _getConfidenceDest() override;

private:
    bool process();

    bool bNewImgRGB;
    bool bNewImgDepthRaw;
    Signal* psigImages;
    image imgRGB;
    cameraFrame_depth *pDepthRaw;
    Mutex mtImages;
    detectNet* pNetwork;
};

class Navigator_SimpleDepthObstruct:public Navigator {
public:
    Navigator_SimpleDepthObstruct();
    virtual ~Navigator_SimpleDepthObstruct();
    
    virtual void newImageDepthRaw(cameraFrame_depth depthRaw, bool bSignal) override;

    virtual bool getObstructions(std::vector<Obstruction>&) override;

    virtual void _run(Signal* psigMain) override;

private:
    bool process();

    bool bNewImgDepthRaw;
    Signal* psigImages;
    cameraFrame_depth *pDepthRaw;
    Mutex mtImages;
    std::vector<Obstruction> arObs;
};
