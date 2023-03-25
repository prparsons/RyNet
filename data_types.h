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

class coord {
public:
    coord();
    coord(const coord& other);

    float fLatitude;
    float fLongitude;
};

inline coord::coord()
{
    fLatitude = 0;
    fLongitude = 0;
}

inline coord::coord(const coord& other)
: fLatitude(other.fLatitude)
, fLongitude(other.fLongitude)
{
}