#pragma once

#include <opencv2/opencv.hpp>   // Include OpenCV API

#define PI 3.141592653590
#define PI_2 1.570796326794
#define PIx2 6.283185307180

typedef cv::Mat image;

class ServerLoop;

class Destination {
public:
    Destination();

    bool operator==(const Destination& other);
    bool operator!=(const Destination& other);

    float fX, fY, fZ;
};

class DestinationEquation {
public:
    DestinationEquation();
    DestinationEquation(float fA, float fB, float fC, float fD, float fE, float fF);
    DestinationEquation(float fXSin, float fXCos, float fC, float fYSin, float fYCos, float fF, bool);

    Destination GetLocation(float fT) const;
    void GetDeriv(float& fDx, float& fDy, float fT) const;
    float GetRouteLength() const { return 0; }

    float fT;
    float fT_min;
    float fT_max;

private:
    bool bQuadratic;
    float fA, fB, fC; // x(t)
    float fD, fE, fF; // y(t)
    float fXSin, fXCos; // x(t) = fXSin * sin(t) + fXCos * cos(t) + fC
    float fYSin, fYCos; // y(t) = fYSin * sin(t) + fYCos * cos(t) + fF

    friend class ServerLoop; // So the status window can write out things
};

// Obstructions should always assume current position is at 0,0
class Obstruction {
public:
    Obstruction(float* arDepthBuffer, int iWidth, int iHeight, float fHorizFOV);
    ~Obstruction();

    bool IsObstructed(const DestinationEquation&, float fTStart, float fOffsetX = 0, float fOffsetY = 0); // returns confidence of obstruction 0 - 255
    void setFloor(int _iFloor) { iFloor = _iFloor; }

private:
    float* arObstructed;
    float* arAngles;
    float fHorizFOV;
    int iWidth;
    int iHeight;
    int iFloor;
};

class Orientation {
public:
    Orientation(): fPitch(0), fRoll(0), fYaw(0) {}

    float fPitch;
    float fRoll;
    float fYaw;
};


class VertexList;

class Vertex {
public:
    explicit Vertex(DestinationEquation& route, Destination destFinal);
    explicit Vertex(Vertex& parent, DestinationEquation& route);

    float GetDistance() const;
    DestinationEquation GetRoute() const { return route; }
    Vertex* GetParent() const { return pParent; }

private:
    Vertex* pParent;
    DestinationEquation route;
    float fDistanceToDest;
    Destination destFinal;

    bool operator<(const Vertex& other) const;
    bool operator>(const Vertex& other) const;
    Vertex* pListNext;
    Vertex* pListPrev;
    friend class VertexList;
};

class VertexList {
public:
    VertexList();
    ~VertexList();

    void add(Vertex* pNew);
    Vertex* pop();

    bool isEmpty() const { return pHead == nullptr; }

private:
    Vertex* pHead;
    Vertex* pPrevInsert;
};
