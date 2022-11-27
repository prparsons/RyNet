#pragma once

#include <arm_neon.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <ctime>
#include <jetson-inference/detectNet.h>
#include <jetson-inference/segNet.h>
#include <jetson-utils/loadImage.h>
#include <vector>

#define PI 3.141592653590
#define PI_2 1.570796326794
#define PIx2 6.283185307180

typedef cv::Mat image;

class DetectionBox {
public:
    DetectionBox();
    DetectionBox(const DetectionBox& other);

    DetectionBox& operator=(const DetectionBox& other);

    float fTop;
    float fRight;
    float fBottom;
    float fLeft;
};

class Destination {
public:
    Destination();

    bool operator==(const Destination& other);
    bool operator!=(const Destination& other);

    float fX, fY, fZ;
};

class Point {
public:
    Point():fX(0),fY(0){}
    Point(float _fX, float _fY):fX(_fX), fY(_fY){}
    Point(const Point& other){ *this = other; }

    Point& operator=(const Point& other) { fX = other.fX; fY = other.fY; }

    float fX;
    float fY;
};

