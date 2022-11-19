#pragma once

#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <ctime>

#define PI 3.141592653590
#define PI_2 1.570796326794
#define PIx2 6.283185307180

class image:public cv::Mat {
public:
    time_t tStamp;
};


class DetectionBox {
public:
    DetectionBox():fTop(0), fRight(0), fBottom(0), bLeft(0){}

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

