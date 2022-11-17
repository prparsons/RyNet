#pragma once

#include <vector>
#include "cameraMulti.h"
#include "data_types.h"
#include "dataLogger.h"

class detectNet;


class ObjectDetection {
public:
    ObjectDetection(DataLogger& logger);

    void ProcessImage();

    DataLogger& logger;
};

class TerrainSegmentation {
public:
    TerrainSegmentation(DataLogger& logger);

    void ProcessImage();

    DataLogger& logger;
};

class Navigator {
public:
    Navigator(DataLogger& logger);

    void runNavigator();
    void runObjectDetect();
    void runTerrainSegment();

private:
    DataLogger& logger;
    std::thread* pThreadNav;
    std::thread* pThreadDet;
    std::thread* pThreadSeg;
    ObjectDetection detector;
    TerrainSegmentation segmenter;
};

