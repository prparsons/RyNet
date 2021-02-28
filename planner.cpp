#include "planner.h"

#include <vector>

// pi / 2
#define Drive_TwinTrackAdv_BRANCH_DIST 1.570796326794

class VertexList;

class Vertex {
public:
    explicit Vertex(DestinationEquation& route, Destination destFinal);
    explicit Vertex(Vertex& parent, DestinationEquation& route);

    float GetDistance() const;
    DestinationEquation GetRoute() const { return route; }

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


// ************
// DrivePlanner

DrivePlanner::DrivePlanner(std::vector<Navigator*>& _arNav)
: arNav(_arNav)
, pDesFinal(nullptr)
, fSpeed(0)
, fSteer(0)
, fClimb(0)
{
}

// ***************
// Drive_TwinTrack

Drive_TwinTrack::Drive_TwinTrack(std::vector<Navigator*>& arNav)
: DrivePlanner(arNav)
, arSteerAvg(5)
{
    for (int i = 0; i < 5; i++) {
        arSteerAvg[i] = 0;
    }
}

bool Drive_TwinTrack::isPlanValid(Navigator& nav, Navigator::dirty)
{
    printf("Drive:%.1f;%.1f  Nav:%.1f;%.1f\n", fSpeed, fSteer, nav.getSpeed(), nav.getOrientation().fYaw);
    bool bReturn = (fSpeed == nav.getSpeed()) && (fSteer == nav.getOrientation().fYaw);
    if (bReturn) {
        printf("isPlanValid(true)\n");
        return true;
    }
    printf("isPlanValid(false)\n");
    return false;
}

void Drive_TwinTrack::createNewPlan()
{
    printf("Drive_TwinTrack::createNewPlan\n");
    // Tell the nav it isn't dirty anymore
    float _fSteer = arNav[0]->getOrientation().fYaw;
    float _fSpeed = arNav[0]->getSpeed();

    fSpeed = _fSpeed;
    fSteer = _fSteer;
}

// ******************
// Drive_TwinTrackAdv

Drive_TwinTrackAdv::Drive_TwinTrackAdv(std::vector<Navigator*>& arNav)
: DrivePlanner(arNav)
{
}

bool Drive_TwinTrackAdv::isPlanValid(Navigator& nav, Navigator::dirty)
{
    if (pDesFinal == nullptr) {
        // We have no destination (or plan) yet, so need to create one
        return false;
    }
    Destination destDesired = nav.getDest();
    if (nav.getDest() != pDesFinal->GetLocation(1)) {
        // Nav is wanting to go somewhere else
        return false; // is this where my averaging stuff goes? YES
        //if dest did change handle tranlation for dest since current location will be new origin
    }
    // Still going to the same place, so check to see if our route is obstructed
    std::vector<Obstruction> arObst;
    if (!nav.getObstructions(arObst)) {
        // There are no obstructions, so done?
        return true;
    }
    for (int i = 0; i < arRoute.size(); i++) {
        if (isRouteObstructed(arRoute[i], arObst)) {
            return false;
        }
    }
    // No obstructions, so done?
    return true;
}

void Drive_TwinTrackAdv::createNewPlan()
{
    printf("Drive_TwinTrackAdv::_decide\n");
    // Tell the nav it isn't dirty anymore
    float _fSteer = arNav[0]->getOrientation().fYaw;
    float _fSpeed = arNav[0]->getSpeed();

    fSpeed = _fSpeed;
    fSteer = _fSteer;
    /////////////////
    //see if nav wants to scale speed

    // Get dest from averaging thing
    // NEED TO SAVE CURRENT DIRECTION!!!
    Destination dest;
    float fDx;
    float fDy;
    DestinationEquation straightShot = createRoute(Destination(), fDx, fDy, dest);
    std::vector<Obstruction> arObst;
    if (!arNav[0]->getObstructions(arObst)) { // not checking validity
        // There are no obstructions, we're good to go
        return;
    }
    if (!isRouteObstructed(straightShot, arObst)) {
        // Route is not obstructed, good to go
        return;
    }
    
    buildRouteTree(dest, arObst);
}

DestinationEquation Drive_TwinTrackAdv::createRoute(Destination start, float fDx, float fDy, Destination dest)
{
    // Cheating right here and not matching derivatives
    //x = bt + c
    //start = c
    // end = bt + c   end - c = b
    DestinationEquation cheater(0, dest.fX - start.fX, start.fX, 0, dest.fY - start.fY, start.fY);
    return cheater;
}

bool Drive_TwinTrackAdv::isRouteObstructed(DestinationEquation destEq, std::vector<Obstruction>& arObst)
{
    for (int i = 0; i < arObst.size(); i++) {
        if (arObst[i].IsObstructed(destEq, destEq.fT)) {
            return true;
        }
    }
    return false;
}

void Drive_TwinTrackAdv::buildRouteTree(Destination dest, std::vector<Obstruction>& arObst)
{
    std::vector<Vertex*> vertList;
    VertexList leafList;
    std::vector<Vertex*> solutionList;
// NEED TO get CURRENT DIRECTION!!!
    float fDx;
    float fDy;
    DestinationEquation start; // match tangents
    Vertex vertParent(start, dest);
    Vertex** arChilds = branch(vertParent);
    //verify they all are valid, add to leafList
    for (int i = 0; i < 3; i++) {
        if (isRouteObstructed(arChilds[i]->GetRoute(), arObst)) {
            delete arChilds[i];
        } else {
            leafList.add(arChilds[i]);
        }
    }
    while (!leafList.isEmpty() && solutionList.size() == 0) {
        Vertex* pVert = leafList.pop();
        arChilds = branch(*pVert);
        for (int i = 0; i < 3; i++) {
            // Check validity
            if (isRouteObstructed(arChilds[i]->GetRoute(), arObst)) delete arChilds[i];

            // See if it has unobstructed view of dest
            DestinationEquation potentialSolution = createRoute(arChilds[i]->GetRoute().GetLocation(1.0), 1, 1, dest);
            if (!isRouteObstructed(potentialSolution, arObst)) {
                solutionList.push_back(arChilds[i]);
            } else {
                //insert into leaf list smallest at the back
                leafList.add(arChilds[i]);
            }
        }
        vertList.push_back(pVert);
    }

    // We have a solution, find the shortest one
    int iShortest = 1000000;
    int iShortestIndex = 0;
    for (int i = 0; i < solutionList.size(); i++) {
        DestinationEquation potentialSolution = createRoute(solutionList[i]->GetRoute().GetLocation(1.0), 1, 1, dest);
        if (solutionList[i]->GetDistance() + potentialSolution.GetRouteLength() < iShortest) {
            iShortestIndex = i;
        }
    }
    
    // get rid of longer solutions
    for (int i = 0; i < solutionList.size(); i++) {
        if (iShortestIndex != i) delete solutionList[i];
    }
    //delete all the other leafs
    while (!leafList.isEmpty()) {
        Vertex* pVert = leafList.pop();
        delete pVert;
    }

    if (pSolution != nullptr) delete pSolution;
    while (arSolutionParentList.size() > 0) {
        delete arSolutionParentList[arSolutionParentList.size() - 1];
        arSolutionParentList.pop_back();
    }
    arSolutionParentList = vertList;
    pSolution = solutionList[iShortestIndex];
}

Vertex** Drive_TwinTrackAdv::branch(Vertex& vert)
{
    // Where we are
    DestinationEquation destEqCurrent = vert.GetRoute();
    Destination destCurrent = destEqCurrent.GetLocation(1); // t ranges from 0-1, so t=1 is the end of that route
    // Where we are pointing
    float fDx;
    float fDy;
    destEqCurrent.GetDeriv(fDx, fDy, 1);
    // Where we are going
    float fX[3];
    float fY[3];
    Vertex** lVertices = new Vertex*[3];

    // Straight
    float f = sqrt(Drive_TwinTrackAdv_BRANCH_DIST * Drive_TwinTrackAdv_BRANCH_DIST) / sqrt((fDx * fDx) + (fDy * fDy));
    fX[0] = destCurrent.fX + (fDx * f);
    fY[0] = destCurrent.fY + (fDy * f);
    DestinationEquation straight(0, destCurrent.fX - fDx, fDx, 0, destCurrent.fY - fDy, fDy);
    lVertices[0] = new Vertex(vert, straight);

            //**not implementing distance here
    if (fDx == 0) {
        // going up or down (90 degree turns)
        if (fDy == 1) {
            // Going up
            // left
            DestinationEquation left(0, 1, destCurrent.fX - 1, 1, 0, destCurrent.fY, true);
            lVertices[1] = new Vertex(vert, left);
            // right
            DestinationEquation right(0, -1, destCurrent.fX + 1, 1, 0, destCurrent.fY, true);
            lVertices[2] = new Vertex(vert, right);
        } else {
            // Going down
            DestinationEquation left(0, -1, destCurrent.fX + 1, -1, 0, destCurrent.fY, true);
            lVertices[1] = new Vertex(vert, left);
            DestinationEquation right(0, 1, destCurrent.fX - 1, -1, 0, destCurrent.fY, true);
            lVertices[2] = new Vertex(vert, right);
        }
    } else {
        // Going left or right
        if (fDx == 1) {
            // Going right
            DestinationEquation left(1, 0, destCurrent.fX, 0, -1, destCurrent.fY + 1, true);
            lVertices[1] = new Vertex(vert, left);
            DestinationEquation right(1, 0, destCurrent.fX, 0, 1, destCurrent.fY - 1, true);
            lVertices[2] = new Vertex(vert, right);
        } else {
            // going left
            DestinationEquation left(-1, 0, destCurrent.fX, 0, 1, destCurrent.fY - 1, true);
            lVertices[1] = new Vertex(vert, left);
            DestinationEquation right(-1, 0, destCurrent.fX, 0, -1, destCurrent.fY + 1, true);
            lVertices[2] = new Vertex(vert, right);
        }
    }

    return lVertices;
}
