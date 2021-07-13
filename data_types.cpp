#include "data_types.h"

#include <cmath>

DetectionBox::DetectionBox(const DetectionBox& other)
{
    *this = other;
}

DetectionBox& DetectionBox::operator=(const DetectionBox& other)
{
    fTop = other.fTop;
    fRight = other.fRight;
    fBottom = other.fBottom;
    fLeft = other.fLeft;
    fDistance = other.fDistance;
}

// ***********
// Destination

Destination::Destination()
: fX(0)
, fY(0)
, fZ(0)
{

}

bool Destination::operator==(const Destination& other)
{
    return (int(fX * 1000) == int(other.fX * 1000)) && (int(fY * 1000) == int(other.fY * 1000)) && (int(fZ * 1000) == int(other.fZ * 1000));
}

bool Destination::operator!=(const Destination& other)
{
    return !(*this == other);
}

// *******************
// DestinationEquation

DestinationEquation::DestinationEquation()
: fT(0)
, fT_min(0)
, fT_max(1)
, bQuadratic(false)
, fA(0)
, fB(0)
, fC(0)
, fD(0)
, fE(0)
, fF(0)
, fXSin(0)
, fXCos(0)
, fYSin(0)
, fYCos(0)
{

}

DestinationEquation::DestinationEquation(float _fA, float _fB, float _fC, float _fD, float _fE, float _fF)
: fT(0)
, fT_min(0)
, fT_max(1)
, bQuadratic(true)
, fA(_fA)
, fB(_fB)
, fC(_fC)
, fD(_fD)
, fE(_fE)
, fF(_fF)
, fXSin(0)
, fXCos(0)
, fYSin(0)
, fYCos(0)
{

}

DestinationEquation::DestinationEquation(float _fXSin, float _fXCos, float _fC, float _fYSin, float _fYCos, float _fF, bool)
: fT(0)
, fT_min(0)
, fT_max(PI_2)
, bQuadratic(false)
, fXSin(0)
, fXCos(0)
, fC(_fC)
, fYSin(0)
, fYCos(0)
, fF(_fF)
{

}

Destination DestinationEquation::GetLocation(float fT) const
{
    Destination dest;
    if (bQuadratic) {
        const float fT2 = fT * fT;
        dest.fX = (fA * fT2) + (fB * fT) + fC;
        dest.fY = (fD * fT2) + (fE * fT) + fF;
    } else {
        const float fSin = sin(fT);
        const float fCos = cos(fT);
        dest.fX = (fXSin * fSin) + (fXCos * fCos) + fC;
        dest.fY = (fYSin * fSin) + (fYCos * fCos) + fF;
    }

    return dest;
}

void DestinationEquation::GetDeriv(float& fDx, float& fDy, float fT) const
{
    if (bQuadratic) {
        fDx = (2 * fA * fT) + fB;
        fDy = (2 * fD * fT) + fE;
    } else {
        const float fSin = sin(fT);
        const float fCos = cos(fT);
        fDx = (fXSin * fCos) - (fXCos * fSin);
        fDy = (fYSin * fCos) - (fYCos * fSin);
    }
}

// ***********
// Obstruction

Obstruction::Obstruction(DetectionBox box, float fTotalWidth, float fHorizFOV)
{
    float fDegreePerPixel = (fHorizFOV / 2) / fTotalWidth;
    float fLeft = box.fLeft - (fTotalWidth / 2);
    fLeft *= fDegreePerPixel;
    fLeft *= (PI / 180); // Convert to radians
    ptLine[0].fX = box.fDistance * sin(fLeft);
    ptLine[0].fY = box.fDistance * cos(fLeft);

    float fRight = box.fRight - (fTotalWidth / 2);
    fRight *= fDegreePerPixel;
    fRight *= (PI / 180); // Convert to radians
    ptLine[1].fX = box.fDistance * sin(fRight);
    ptLine[1].fY = box.fDistance * cos(fRight);
}

Obstruction::~Obstruction()
{

}

bool Obstruction::IsObstructed(const DestinationEquation& destEq, float fTStart, float fOffsetX, float fOffsetY)
{
    if (ptLine[0].fX == ptLine[1].fX) {
        // Should make this parametric, cheating for now since this shouldn't be possible anyways
        return false;
    }
    Destination destPrev = destEq.GetLocation(fTStart);

    // y = mx + b  for the obstruction
    float fObsM = (ptLine[1].fY - ptLine[0].fY) / (ptLine[1].fX - ptLine[0].fX);
    float fObsB = ptLine[0].fY - (fObsM * ptLine[0].fX);

    for (float fT = fTStart + .1; fT <= 1.0; fT += .1) {
        Destination dest = destEq.GetLocation(fT);
        if (dest.fX == destPrev.fX) {
            // Straight line
            if (dest.fX < ptLine[0].fX || dest.fX > ptLine[1].fX) {
                // Obstruction not in this line of travel
                continue;
            }
            float fIntersection = (fObsM * dest.fX) + fObsB;
            if (destPrev.fY < fIntersection && fIntersection < dest.fY) {
                // Intersection is within the segment of travel
                return true;
            }
            continue;
        }
        // y = mx + b for that segment of travel
        float fM = (dest.fY - destPrev.fY) / (dest.fX - destPrev.fX);
        float fB = destPrev.fY - (fM * destPrev.fX);
        if (fObsM == fM) {
            if (fB == fObsB) return true; // They are the same line
            continue; // Parallel lines
        }
        float fXintersect = (fB - fObsB) / (fObsM - fM);
        if (ptLine[0].fX < fXintersect && fXintersect < ptLine[1].fX) {
            float fLeft, fRight;
            if (dest.fX < destPrev.fX) {
                fLeft = dest.fX;
                fRight = destPrev.fX;
            } else {
                fRight = dest.fX;
                fLeft = destPrev.fX;
            }
            if (fLeft < fXintersect && fXintersect < fRight) {
                return true;
            }
        }
    }
    return false;
}

// ******
// Vertex

Vertex::Vertex(DestinationEquation& _route, Destination _destFinal)
: pParent(nullptr)
, route(_route)
, destFinal(_destFinal)
, pListNext(nullptr)
, pListPrev(nullptr)
{
    Destination start = route.GetLocation(1);
    fDistanceToDest = sqrt( ((destFinal.fX - start.fX) * (destFinal.fX - start.fX)) +
                             ((destFinal.fY - start.fY) * (destFinal.fY - start.fY)) +
                             ((destFinal.fZ - start.fZ) * (destFinal.fZ - start.fZ)));
}

Vertex::Vertex(Vertex& parent, DestinationEquation& _route)
: pParent(&parent)
, route(_route)
, destFinal(parent.destFinal)
{
    //destFinal needs to get translated
    Destination start = route.GetLocation(1);
    fDistanceToDest = sqrt( ((destFinal.fX - start.fX) * (destFinal.fX - start.fX)) +
                             ((destFinal.fY - start.fY) * (destFinal.fY - start.fY)) +
                             ((destFinal.fZ - start.fZ) * (destFinal.fZ - start.fZ)));
}

float Vertex::GetDistance() const
{
    if (pParent != nullptr) return pParent->GetDistance() + fDistanceToDest;
    return fDistanceToDest;
}

bool Vertex::operator<(const Vertex& other) const
{
    return GetDistance() < other.GetDistance();
}

bool Vertex::operator>(const Vertex& other) const
{
    return GetDistance() > other.GetDistance();
}


// ********
// VertexList

VertexList::VertexList()
: pHead(nullptr)
, pPrevInsert(nullptr)
{

}

VertexList::~VertexList()
{

}

void VertexList::add(Vertex* pNew)
{
    // List empty
    if (pHead == nullptr) {
        pHead = pNew;
        return;
    }
    // List not empty
    if (pPrevInsert == nullptr) {
        pPrevInsert = pHead;
    }

    // Move index towards end of array
    while (*pNew > *pPrevInsert) {
        if (pPrevInsert->pListNext == nullptr) break;
        pPrevInsert = pPrevInsert->pListNext;
    }
    // Move index towards front of array
    while (*pNew < *pPrevInsert) {
        if (pPrevInsert->pListPrev == nullptr) break;
        pPrevInsert = pPrevInsert->pListPrev;
    }

    // Check if it's the head
    if (pPrevInsert == nullptr) {
        pNew->pListNext = pHead;
        pHead->pListPrev = pNew;
        return;
    }

    // Insert it after pPrevInsert
    Vertex* pNext = pPrevInsert->pListNext;
    pPrevInsert->pListNext = pNew;
    pNew->pListPrev = pPrevInsert;
    pNew->pListNext = pNext;
    // Set the next node's prev to this
    if (pNext != nullptr) {
        pNext->pListPrev = pNew;
    }
}

Vertex* VertexList::pop()
{
    Vertex* pReturn = pHead;
    if (pReturn == nullptr) return nullptr;
    if (pHead->pListNext == nullptr) {
        pHead = nullptr;
    } else {
        pHead = pHead->pListNext;
        pHead->pListPrev = nullptr;
    }
    return pReturn;
}