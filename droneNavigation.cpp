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
    image imageRGB = pFrame->getImageRGB();
    if (pFrame->hasDepth()) {
        image imageDepth = pFrame->getImageDepth();
        nav.checkObstruct(imageRGB, imageDepth);
        nav.checkAltitude(imageRGB, imageDepth);
    } else {
        nav.checkObstruct(imageRGB);
        nav.checkAltitude(imageRGB);
    }
    if (pFrame->hasIR()) {
        image imageIR = pFrame->getImageIR();
        nav.checkTerrain(imageRGB, imageIR);
    } else {
        nav.checkTerrain(imageRGB);
    }
    nav.checkAnimals(imageRGB);
}

// *****
// Drive

Drive::Drive()
: fSpeed(0)
, fSteer(0)
, fClimb(0)
{
}

// **************************
// Navigator_SimpleObstruct2D

Navigator_SimpleObstruct2D::Navigator_SimpleObstruct2D()
: bIsDirty(false)
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

bool Navigator_SimpleObstruct2D::checkTerrain(image imgRGB)
{
    return false;
}

bool Navigator_SimpleObstruct2D::checkObstruct(image imgRGB, image imgDepth)
{
    return false;
}

void Navigator_SimpleObstruct2D::operator()(Signal)
{
    if (pCamera == nullptr) {
        printf("No camera\n");
        return;
    }
    std::thread thrCamera(*pCamera, (void*)&updateDepthCamera, (void*)this);

    //Nav Loop here

    thrCamera.join();
}

// ***************
// Drive_TwinTrack

bool Drive_TwinTrack::_decide()
{
    return false;
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
    Signal signalNav;
    std::thread thrNav(nav, signalNav);
    bool bDone = false;
    do {
        if (nav.isDirty()) {
            _decide();
            if (bNeedGuidance) {
                _sendGuidance();
                bNeedGuidance = false;
            }
        }
        signalNav.wait(drive.getLoopTimeoutMS());
    } while (!bDone);
}

void DroneNavigation::_decide()
{
    if (drive._decide()) {
        bNeedGuidance = true;
    }
}

void DroneNavigation::_sendGuidance()
{

}
