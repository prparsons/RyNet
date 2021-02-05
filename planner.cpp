#include "planner.h"

#include <vector>


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
{
}

bool Drive_TwinTrack::isPlanValid(Navigator& nav, Navigator::dirty)
{
    return (fSpeed != nav.getSpeed()) || (fSteer != nav.getOrientation());
}

void Drive_TwinTrack::createNewPlan()
{
    printf("Drive_TwinTrack::_decide\n");
    // Tell the nav it isn't dirty anymore
    float _fSteer = arNav[0]->getOrientation();
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
    if (nav.getDest() != pDesFinal->dest) {
        // Nav is wanting to go somewhere else
        return false; // is this where my averaging stuff goes?
    }
    // Still going to the same place, so check to see if our route is obstructed
    std::vector<Obstruction> arObst;
    if (!nav.getObstructions(arObst)) {
        // There are no obstructions, so done?
        return true;
    }
    for (int i = 0; i < arRoute.size(); i++) {
        if (isRouteObstructed(arRoute[i].route, arObst)) {
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
    float _fSteer = arNav[0]->getOrientation();
    float _fSpeed = arNav[0]->getSpeed();

    fSpeed = _fSpeed;
    fSteer = _fSteer;
}

DestinationEquation Drive_TwinTrackAdv::createRoute(float fX, float fY)
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
