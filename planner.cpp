#include "planner.h"

#include <vector>


// ************
// DrivePlanner

DrivePlanner::DrivePlanner(std::vector<Navigator*>& _arNav)
: arNav(_arNav)
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

bool Drive_TwinTrack::_decide(Navigator&, Navigator::dirty)
{
    return _decide();
}

bool Drive_TwinTrack::_decide()
{
    printf("Drive_TwinTrack::_decide\n");
    // Tell the nav it isn't dirty anymore
    float _fSteer = arNav[0]->getOrientation();
    float _fSpeed = arNav[0]->getSpeed();

    bool bReturn = (fSpeed != _fSpeed) || (fSteer != _fSteer);

    fSpeed = _fSpeed;
    fSteer = _fSteer;
    return bReturn;
}
