#pragma once

#include <thread>

typedef int image;

class cameraFrame {
public:
    image getImageRGB();
    image getImageDepth();
    image getImageIR();
    image getImageREdge();

    bool hasRGB() const;
    bool hasDepth() const;
    bool hasIR() const;
    bool hasREdge() const;

private:
    // whatever needed for each type
};

class cameraMulti {
public:
    enum cameraType {
        cameraMulti_CV,
        cameraMulti_RS };

    enum cameraFeatures {
        cameraMulti_RGB = 0x01,
        cameraMulti_IR = 0x02,
        cameraMulti_Depth = 0x04,
        cameraMulti_REdge = 0x08 };

    static cameraMulti* OpenCamera(cameraType type, cameraFeatures features);
    static bool isCameraTypeAvail(cameraType type);

    int getAvailFeatures() const;

    void runCameraLoop();
    cameraFrame* waitForCamera();
    void operator()(void* pCallbackFrame, void* pCallbackData);

protected:
    cameraMulti(cameraType);

private:
    void (*callbackFrame)(cameraFrame*, void*);
    void* pCallbackData;
};

inline cameraMulti* cameraMulti::OpenCamera(cameraType type, cameraFeatures features)
{
    return nullptr;
}

inline void cameraMulti::runCameraLoop()
{
    while (true) {
        cameraFrame* pFrame = nullptr;
        if (callbackFrame != nullptr) {
            callbackFrame(pFrame, pCallbackData);
        }
    }
}

