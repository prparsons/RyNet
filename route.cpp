#include "route.h"

#include <string.h>
#include "dataLogger.h"

CoordNode::CoordNode(float _fLat, float _fLon)
: fLat(_fLat)
, fLon(_fLon)
, nw(nullptr)
, ne(nullptr)
, se(nullptr)
, sw(nullptr)
{
}

CoordNode::~CoordNode()
{
    DestroyChildren();
}

float CoordNode::GetDistance(float fLatQuery, float fLonQuery) const
{
    return 1.0;
}

const CoordNode* CoordNode::GetClosest(float fLatQuery, float fLonQuery, float& fDistance) const
{
    fDistance = GetDistance(fLatQuery, fLonQuery);
    float fClosestChild = 9999999;
    const CoordNode* pNode = nullptr;

    if (fLatQuery > fLat) {
        // It's north of this node center
        if (fLonQuery > fLon) {
            // It's west of this node center
            pNode = nw->GetClosest(fLatQuery, fLonQuery, fClosestChild);
            
        } else {
            // It's east of this node center
            pNode = ne->GetClosest(fLatQuery,fLonQuery, fClosestChild);
        }
    } else {
        // It's south of this node center
        if (fLonQuery > fLon) {
            // It's west of this node center
            pNode = sw->GetClosest(fLatQuery, fLonQuery, fClosestChild);
        } else {
            // It's east of this node center
            pNode = se->GetClosest(fLatQuery, fLonQuery, fClosestChild);
        }
    }

    if (pNode != nullptr) {
        if (fClosestChild < fDistance) {
            fDistance = fClosestChild;
            return pNode;
        }
    }
    return this;
}

void CoordNode::InsertPoint(float fLatNew, float fLonNew)
{
  if (fLatNew > fLat) {
        // It's north of this node center
        if (fLonNew > fLon) {
            // It's west of this node center
            if (nw == nullptr) {
                nw = new CoordNode(fLatNew, fLonNew);
            } else {
                nw->InsertPoint(fLatNew, fLonNew);
            }
        } else {
            // It's east of this node center
            if (ne == nullptr) {
                ne = new CoordNode(fLatNew, fLonNew);
            } else {
                ne->InsertPoint(fLatNew, fLonNew);
            }
        }
    } else {
        // It's south of this node center
        if (fLonNew > fLon) {
            // It's west of this node center
            if (sw == nullptr) {
                sw = new CoordNode(fLatNew, fLonNew);
            } else {
                sw->InsertPoint(fLatNew, fLonNew);
            }
        } else {
            // It's east of this node center
            if (se == nullptr) {
                se = new CoordNode(fLatNew, fLonNew);
            } else {
                se->InsertPoint(fLatNew, fLonNew);
            }
        }
    }
}

void CoordNode::ResetPosition(float _fLat, float _fLon)
{
    DestroyChildren();
    fLat = _fLat;
    fLon = _fLon;
}

void CoordNode::DestroyChildren()
{
    if (ne != nullptr) {
        delete ne;
        ne = nullptr;
    }
    if (se != nullptr) {
        delete se;
        se = nullptr;
    }
    if (sw != nullptr) {
        delete sw;
        sw = nullptr;
    }
    if (nw != nullptr) {
        delete nw;
        nw = nullptr;
    }
}

void CoordNode::Print() const
{
    printf("{%.3f,%.3f\nNE:", fLat, fLon);
    if (ne != nullptr) ne->Print();
    else printf("nullptr\n");
    
    printf("SE:");
    if (se != nullptr) se->Print();
    else printf("nullptr\n");
    
    printf("SW:");
    if (sw != nullptr) sw->Print();
    else printf("nullptr\n");
    
    printf("NW:");
    if (nw != nullptr) nw->Print();
    else printf("nullptr\n");

    printf("}\n");
}

Route::Route(char* buffer, Route& prev)
{
    memcpy(&cmd, buffer, sizeof(command));
    prev.pNext = this;
    this->pNext = nullptr;
}
Route::Route(char* buffer, size_t uBufferLen)
{
    pNext = nullptr;
    const size_t sizeCmd = sizeof(command);
    if (uBufferLen % sizeCmd != 0) {
        printL(" Invalid buffer");
        pNext = nullptr;
        return;
    }

    Route* pPrev = this;
    memcpy(&cmd, buffer, sizeof(command));
    for (size_t indexBuffer = sizeCmd; indexBuffer < uBufferLen; indexBuffer += sizeCmd) {
        pPrev = new Route(buffer + indexBuffer, *pPrev);
    }
}
Route::~Route()
{
    if (pNext != nullptr) delete pNext;
}

std::string Route::toString() const
{
    char stringstreamSucks[256];

    snprintf(stringstreamSucks, 256, "%d %d %d %d %.1f %.1f %.1f %.1f %.6f %.6f %.1f %d",
        int(cmd.seq),
        int(cmd.current),
        int(cmd.frame),
        int(cmd.command),
        cmd.param1,
        cmd.param2,
        cmd.param3,
        cmd.param4,
        cmd.lat,
        cmd.lon,
        cmd.alt,
        int(cmd.autocontinue));
    std::string returnVal(stringstreamSucks);

    if (pNext != nullptr) return returnVal + "\n" + pNext->toString();
    return returnVal;
}