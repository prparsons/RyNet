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

CoordNode::CoordNode(CoordNode& other, float _fLat, float _fLon)
: fLat(other.fLat)
, fLon(other.fLon)
, nw(other.nw)
, ne(other.ne)
, se(other.se)
, sw(other.sw)
{
    other.fLat = _fLat;
    other.fLon = _fLon;
    other.nw = nullptr;
    other.ne = nullptr;
    other.se = nullptr;
    other.sw = nullptr;
}

CoordNode::~CoordNode()
{
    DestroyChildren();
}

float CoordNode::GetDistance(float fLatQuery, float fLonQuery) const
{
    return PointDistanceM(fLat, fLon, fLatQuery, fLonQuery);
}

const CoordNode* CoordNode::GetClosest(float fLatQuery, float fLonQuery) const
{
    float f;
    return GetClosest(fLatQuery, fLonQuery, f);
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
            if (nw != nullptr) pNode = nw->GetClosest(fLatQuery, fLonQuery, fClosestChild);
            
        } else {
            // It's east of this node center
            if (ne != nullptr) pNode = ne->GetClosest(fLatQuery,fLonQuery, fClosestChild);
        }
    } else {
        // It's south of this node center
        if (fLonQuery > fLon) {
            // It's west of this node center
            if (sw != nullptr) pNode = sw->GetClosest(fLatQuery, fLonQuery, fClosestChild);
        } else {
            // It's east of this node center
            if (se != nullptr) pNode = se->GetClosest(fLatQuery, fLonQuery, fClosestChild);
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

Route::Route(const Route& other)
{
    cmd = other.cmd;
    pNext = nullptr;
    if (!other.pNext) return;

    const Route* pIndexOther = other.pNext;
    Route* pIndexNew = this;
    do {
        pIndexNew->pNext = new Route(pIndexOther->cmd);
        pIndexNew->pNext->pNext = nullptr;
        pIndexNew = pIndexNew->pNext;
        pIndexOther = pIndexOther->pNext;
    } while(pIndexOther != nullptr);
}

Route::Route(const command& cmdCopy)
{
    cmd = cmdCopy;
    pNext = nullptr;
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

float Route::GetDistance() const
{
    if (pNext == nullptr) return 0;
    float fDistance = PointDistanceM(cmd.lat, cmd.lon, pNext->cmd.lat, pNext->cmd.lon);
    return fDistance + pNext->GetDistance();
}
#include <time.h>
Route* Route::Tsp2Opt(Route* pStart, CoordNode* pVisitedPoints, int16_t iNextPoint)
{
    if (pStart == nullptr) return nullptr;

    // Get rid of waypoints we've already been to
    pStart = Route::removeVisited(pStart, iNextPoint);


    //@todo get rid of non-commands
    CoordNode cheat(39.45893080,	-105.04784280);
    // If we have a list of points we've been to in guided, use that to clear out any waypoints
    // that have already been "visited".
    if (pVisitedPoints) {
        pStart = Route::cleanRoute(pStart, pVisitedPoints);
    }

    // // Now run the 2Opt algorithm on the remaining path
    float fCurrentDistance = pStart->GetDistance();
    clock_t start = clock();
    Route* pNewRoute = single2Opt(*pStart);
    float fNewDistance = pNewRoute->GetDistance();
    while (fNewDistance / fCurrentDistance < 1.0) {
        Route* pTemp = pStart;
        pStart = pNewRoute;
        delete pTemp;
        fCurrentDistance = fNewDistance;

        pNewRoute = single2Opt(*pStart);
        fNewDistance = pNewRoute->GetDistance();
    }
    clock_t end = clock();
    printf("Time for single 2opt: %f\n", float(end - start) / CLOCKS_PER_SEC);
    printL(pNewRoute->toString().c_str());
    delete pNewRoute;
    return pStart;
}

Route* Route::cleanRoute(Route* pStart, CoordNode* pVisitedPoints)
{
    if (pStart == nullptr) return nullptr;
    if (pVisitedPoints == nullptr) return nullptr;
    printf("cleanRoute pstart %p\n", pStart);
    std::queue<Route*> closeWaypoints;
    float fTooClose = 100;
    // First start off by checking each waypoint against the visited list
    for (Route* pThis = pStart; pThis != nullptr; pThis = pThis->pNext) {
        const CoordNode* pClosestPoint = pVisitedPoints->GetClosest(pThis->cmd.lat, pThis->cmd.lon);
        if (pClosestPoint == nullptr) {
            printL("no closest point?");
        }
        if (pClosestPoint->GetDistance(pThis->cmd.lat, pThis->cmd.lon) < fTooClose) {
            closeWaypoints.push(pThis);
        }
    }
        printf("Close waypoints size: %d\n", int(closeWaypoints.size()));
    if (closeWaypoints.size() == 0) {
        printL("No points to remove, returning");
        return pStart;
    }
    printL("done checking list");
    // Start off seeing if we need to advance pStart as we delete the first things on the list
    while (closeWaypoints.front() == pStart) {
        pStart = pStart->pNext;

        closeWaypoints.front()->pNext = nullptr;
        delete closeWaypoints.front();
        closeWaypoints.pop();
    }
    printL("done seeing if we need to advance pStart");

    // Then just look for pNext pointers that match the queue
    for (Route* pThis = pStart; pThis != nullptr && closeWaypoints.front() != nullptr;) {
        if (pThis->pNext == closeWaypoints.front()) {
            pThis->pNext = pThis->pNext->pNext;

            closeWaypoints.front()->pNext = nullptr;
            delete closeWaypoints.front();
            closeWaypoints.pop();
            continue; // Don't want to advance pThis since the new pNext might need to be deleted too
        }
        pThis = pThis->pNext;
    }
    printL("done cleanRoute");
    return pStart;
}

Route* Route::removeVisited(Route* pStart, int16_t iNextPoint)
{
    if (pStart == nullptr) return nullptr;
    printL("removeVisited");
    while (pStart && pStart->cmd.seq < iNextPoint) {
        Route* pDelete = pStart;
        pStart = pStart->pNext;

        printf("Removing %p; New Head %p\n", pDelete, pStart);
        pDelete->pNext = nullptr;
        delete pDelete;
    }
    return pStart;
}

Route* Route::single2Opt(const Route& start)
{
    Route* pStart = new Route(start);
    std::vector<Route*> routeList;
    for (Route* pIndex = pStart; pIndex != nullptr; pIndex = pIndex->pNext) {
        routeList.push_back(pIndex);
    }

    float fBestDistance = pStart->GetDistance();
    size_t uLast = routeList.size() - 2;
    size_t uFirst = 1;

    for (size_t i = uFirst; i < uLast - 1; i++) {
        for (size_t j = i + 2; j < uLast; j++) {
            size_t beg_index = i;
            size_t end_index = j;
            //printf("Left:%d    Right:%d\n", int(beg_index), int(end_index));

            //swap only the pointers around, leave the vector alone
            for(int left = beg_index; left < end_index; left++) {
                routeList[left + 1]->pNext = routeList[left];
            }
            routeList[beg_index - 1]->pNext = routeList[end_index];
            routeList[beg_index]->pNext = routeList[end_index + 1];

            float fNewDistance = pStart->GetDistance();
            if (fNewDistance < fBestDistance) {
                fBestDistance = fNewDistance;
                printL("Found new best");
                // now swap the vector too since we only swapped the pointers
                for(int left = beg_index, right = end_index; left < right; left++, right--) {
                    Route* temp = routeList[left];
                    routeList[left] = routeList[right];
                    routeList[right] = temp;
                }
                // for (int n = 0; n < routeList.size() - 1; n++) {
                //     if (routeList[n]->pNext != routeList[n+1]) {
                //         printf("Something is wrong[left,right,n]%d %d %d\n", beg_index, end_index, n);
                //         delete pStart;
                //         pStart = new Route(start);
                //         return pStart;
                //     }
                // }
            } else {
                //put the pointers back
                for(int left = beg_index - 1; left <= end_index; left++) {
                    routeList[left]->pNext = routeList[left + 1];
                }
                // for (int n = 0; n < routeList.size() - 1; n++) {
                //     if (routeList[n]->pNext != routeList[n+1]) {
                //         printf("Something is wrong2[left,right,n]%d %d %d\n", beg_index, end_index, n);
                //         delete pStart;
                //         pStart = new Route(start);
                //         return pStart;
                //     }
                // }
            }
        }
    }
    return pStart;
}
