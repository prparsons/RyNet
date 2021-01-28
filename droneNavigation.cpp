#include "droneNavigation.h"

#include <stdio.h>
#include <thread>
#include <unistd.h>
#include "cameraMulti.h"

void updateDepthCamera(cameraFrame* pFrame, void* pNavigator)
{
    if (pFrame == nullptr || pNavigator == nullptr) {
        printf("@updateDepthCamera:arguments were null\n");
        return;
    }
    Navigator& nav = *((Navigator*)pNavigator);
    // hasRGB, hasIR, hasDepth, hasREdge; getImageRGB
    if (!pFrame->hasRGB()) {
        printf("updateDepthCamera:frame has no rgb\n");
        return;
    }
    bool bDepth = pFrame->hasDepth();
    bool bIR = pFrame->hasIR();
    bool bREdge = pFrame->hasREdge();
    image imageRGB = pFrame->getImageRGB();
    nav.newImageRGB(imageRGB, !(bDepth || bIR || bREdge));
    if (bDepth) {
        image imageDepth = pFrame->getImageDepth();
        nav.newImageDepth(imageDepth, !(bIR || bREdge));
    }
    if (bIR) {
        image imageIR = pFrame->getImageIR();
        nav.newImageIR(imageIR, !bREdge);
    } 
    if (bREdge) {
        image imageREdge = pFrame->getImageREdge();
        nav.newImageREdge(imageREdge, true);
    }
}

// *****
// Drive

Drive::Drive(Navigator& _nav)
: nav(_nav)
, fSpeed(0)
, fSteer(0)
, fClimb(0)
{
}

// **************************
// Navigator_SimpleObstruct2D

Navigator_SimpleObstruct2D::Navigator_SimpleObstruct2D()
: bIsDirty(false)
, bNewImgRGB(false)
, bNewImgDepth(false)
, pCamera(nullptr)
{
    cameraMulti::cameraFeatures features = cameraMulti::cameraFeatures(cameraMulti::cameraMulti_RGB | cameraMulti::cameraMulti_Depth);
    pCamera = cameraMulti::OpenCamera(cameraMulti::cameraMulti_RS, features);
    if (pCamera == nullptr) {
        printf("Error opening camera\n");
        return;
    }
}

Navigator_SimpleObstruct2D::~Navigator_SimpleObstruct2D()
{
    if (pCamera != nullptr) {
        delete pCamera;
    }
}

void Navigator_SimpleObstruct2D::newImageRGB(image _imgRGB, bool bSignal)
{
    ThreadLock locker(mtImages);
    locker.lock();
    bNewImgRGB = true;
    imgRGB = _imgRGB; // copying is slow...need a single shareable read instance
    locker.unlock();
    if (bSignal) {
        if (psigImages != nullptr) {
            psigImages->signal();
        }
    }
}

void Navigator_SimpleObstruct2D::newImageDepth(image _imgDepth, bool bSignal)
{
    ThreadLock locker(mtImages);
    locker.lock();
    bNewImgDepth = true;
    imgDepth = _imgDepth;
    locker.unlock();
    if (bSignal) {
        if (psigImages != nullptr) {
            psigImages->signal();
        }
    }
}

float Navigator_SimpleObstruct2D::getObstructDirection()
{
    return 0.0;
}

float Navigator_SimpleObstruct2D::getObstructSpeed()
{
    return 0.0;
}

bool Navigator_SimpleObstruct2D::processTerrain()
{
    return false; // This one isn't bothering with terrain
}

bool Navigator_SimpleObstruct2D::processObstruct()
{
    return false; // This should figure out speed, and turn left if something is in the way
}

bool Navigator_SimpleObstruct2D::processAnimals()
{
    return false; // Use this possibly for altitude?
}

void Navigator_SimpleObstruct2D::operator()(Signal* psigMain)
{
    Signal& sigMain = *psigMain;
    Signal sigImages;
    psigImages = new Signal(sigImages);

    if (pCamera == nullptr) {
        printf("No camera\n");
        return;
    }
    std::thread thrCamera(*pCamera, (void*)&updateDepthCamera, (void*)this);

    bool bDone = false;
    do {
        sigImages.wait();

        if (!(bNewImgDepth && bNewImgRGB)) continue; // Wait until new images of both
        if (processObstruct()) {
            bIsDirty = true;
            sigMain.signal();
        }

        bNewImgDepth = false;
        bNewImgRGB = false;
        bool bSignal = false;
        bSignal = processTerrain();
        bSignal = processAnimals() || bSignal;
        if (bSignal) {
            bIsDirty = true;
            sigMain.signal();
        }
        break;
    } while(!bDone);

    thrCamera.join();
    delete psigMain;
    delete psigImages;
}

// ***************
// Drive_TwinTrack

Drive_TwinTrack::Drive_TwinTrack(Navigator& nav)
: Drive(nav)
{
}

bool Drive_TwinTrack::_decide()
{
    // Tell the nav it isn't dirty anymore
    float _fSteer = nav.getObstructDirection();
    float _fSpeed = nav.getObstructSpeed();

    fSpeed = _fSpeed;
    fSteer = _fSteer;
    return true;
}

// ***************
// DroneNavigation

DroneNavigation::DroneNavigation(Navigator& _nav, Drive& _drive)
: nav(_nav)
, drive(_drive)
{
    printf("DroneNavigation\n");
}

void DroneNavigation::operator()(void)
{
    printf("DroneNavigation Thread\n");
    std::thread thrNav(nav, new Signal(signalMainMessage));
    bool bDone = false;
    do {
        if (nav.isDirty()) {
            nav.setClean();
            _decide();
        }
        signalMainMessage.wait(drive.getLoopTimeoutMS());
    } while (!bDone);
}

void DroneNavigation::_decide()
{
    if (drive._decide()) {
       _sendGuidance();
    }
}

void DroneNavigation::_sendGuidance()
{

}
