#include "cameraMulti.h"

bool cameraFrame::hasRGB() const
{
    return true;
}

bool cameraFrame::hasIR() const
{
    return true;
}

bool cameraFrame::hasDepth() const
{
    return true;
}

bool cameraFrame::hasREdge() const
{
    return false;
}

image cameraFrame::getImageRGB()
{
    return 0;
}

image cameraFrame::getImageDepth()
{
    return 0;
}

image cameraFrame::getImageIR()
{
    return 0;
}

image cameraFrame::getImageREdge()
{
    return 0;
}

void cameraMulti::operator()(void* pCallbackFrame, void* _pCallbackData)
{
    callbackFrame = (void(*)(cameraFrame*, void*))pCallbackFrame;
    pCallbackData = _pCallbackData;
    runCameraLoop();
}
