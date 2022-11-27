#include "data_types.h"

#include <cmath>

DetectionBox::DetectionBox()
: fTop(0)
, fRight(0)
, fBottom(0)
, fLeft(0)
{

}

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
