#pragma once

#include <vector>
#include "cameraMulti.h"
#include "data_types.h"
#include "dataLogger.h"

// Place holders with the jetson inference networks...once that's working...
class detectNet;
class segNet;

class ObjectDetection {
public:
    ObjectDetection(DataLogger& logger);
    void ProcessImage();

    DataLogger& logger;
    detectNet* pNetwork;
};

class TerrainSegmentation {
public:
    TerrainSegmentation(DataLogger& logger);
    void ProcessImage();

    enum Classes {
        SKY = 0,
        TREES,
        WATER,
        TERRAIN,
        HABITAT
    };

private:
    bool ScanMask(uchar3* imgMask, int iWidth, int iHeight);

    DataLogger& logger;
    segNet* pNetwork;
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

