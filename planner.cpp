#include "planner.h"

#include <vector>

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
};

Vertex::Vertex(DestinationEquation& _route, Destination _destFinal)
: pParent(nullptr)
, route(_route)
, destFinal(_destFinal)
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
        //good to go
        return;
    }
    if (!isRouteObstructed(straightShot, arObst)) {
        //good to go
        return;
    }
    
    buildRouteTree(dest, arObst);
}

DestinationEquation Drive_TwinTrackAdv::createRoute(Destination start, float fDx, float fDy, Destination dest)
{
    return DestinationEquation();
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
    std::vector<Vertex*> leafList;
    std::vector<Vertex*> solutionList;
// NEED TO get CURRENT DIRECTION!!!
    float fDx;
    float fDy;
    DestinationEquation start; // match tangents
    Vertex vertParent(start, dest);
    Vertex** arChilds = branch(vertParent);
    //verify they all are valid, add to leafList
    while (leafList.size() > 0) {
        Vertex* pVert = leafList[leafList.size()];
        leafList.pop_back();
        arChilds = branch(*pVert);
        for (int i = 0; i < 3; i++) {
            // Check validity
            if (isRouteObstructed(arChilds[i]->GetRoute(), arObst)) delete arChilds[i];

            // See if it has unobstructed view of dest
            DestinationEquation potentialSolution;// = createRoute(arChilds[i].start, dest)
            if (!isRouteObstructed(potentialSolution, arObst)) {
                solutionList.push_back(arChilds[i]);
            } else {
                //insert into leaf list smallest at the back
                leafList.push_back(arChilds[i]);
            }
        }
        vertList.push_back(pVert);
    }
    //delete all the vertices
}

Vertex** Drive_TwinTrackAdv::branch(Vertex& vert)
{
    return nullptr;
}
