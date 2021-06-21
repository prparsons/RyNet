#include "planner.h"

#include <vector>


#define Drive_TwinTrackAdv_BRANCH_DIST 1.570796326794


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
, pCurrentRoute(nullptr)
, fDistBeforeChangeDest(1000)
, iDestAvgCursor(0)
, iSpeedAvgCursor(0)
, iOrientationAvgCursor(0)
{
}

bool Drive_TwinTrackAdv::isPlanValid(Navigator& nav, Navigator::dirty flags)
{
    if (pCurrentRoute == nullptr) {
        // We have no destination (or plan) yet, so need to create one
        return false;
    }
    
    // Check destination
    if ((flags && Navigator::DEST_X) || (flags && Navigator::DEST_Y) || (flags && Navigator::DEST_Z)) {
        Destination destDesired = nav.getDest();
        if (processDestRunningAvg(destDesired)) {
            // Need to go somewhere else
            return false;
        }
    }
    
    // Still going to the same place, so check to see if our route is obstructed
    std::vector<Obstruction> arObst;
    if (!nav.getObstructions(arObst)) {
        // There are no obstructions, so done?
        return true;
    }
    Destination destCurrent = pCurrentRoute->GetRoute().GetLocation(pCurrentRoute->GetRoute().fT);
    for (int i = 0; i < arRouteList.size(); i++) {
        if (isRouteObstructed(arRouteList[i]->GetRoute(), arObst, -destCurrent.fX, -destCurrent.fY)) {
            return false;
        }
    }
    
    return true;
}

void Drive_TwinTrackAdv::createNewPlan()
{
    printf("Drive_TwinTrackAdv::createNewPlan\n");
    // Tell the nav it isn't dirty anymore
    float _fSteer = arNav[0]->getOrientation().fYaw;
    float _fSpeed = arNav[0]->getSpeed();

    fSpeed = _fSpeed;
    fSteer = _fSteer;
    /////////////////
    // What we need to do here is have a priority system for the nav objects,
    //      get desired speed, steer, destination from any that have them available (they shouldn't have to have everything)
    //      use priority for conflicts
    //      speed doesn't affect destination
    //      if there's no destination, any steer is fine
    //      if there is a destination, any higher priority steer should cancel it? (or save it for now?)
    //      i think priority should be decided when they are being added, and each (speed, steer, dest)
    //          should have it's own priority (fail if something not set)

    
    Destination dest = destAvg;
    float fDx;
    float fDy;
    // For this, need to rotate dest
    DestinationEquation straightShot = createRoute(Destination(), fDx, fDy, dest);
    std::vector<Obstruction> arObst;
    bool bObstructionsExist = arNav[0]->getObstructions(arObst);
    bool bRouteObstructed = false;
    if (bObstructionsExist) {
        bRouteObstructed = isRouteObstructed(straightShot, arObst);
    }
    if ((!bObstructionsExist) || (!bRouteObstructed)) {
        // There are no obstructions, we're good to go
        // Clear the current route list
        while (arRouteList.size() > 0) {
            delete arRouteList[arRouteList.size() - 1];
            arRouteList.pop_back();
        }
        arRouteList[0] = new Vertex(straightShot, dest);
        arRouteList.shrink_to_fit();
        return;
    }
    // There are obstructions, build a route
    buildRouteTree(dest, arObst);
}

DestinationEquation Drive_TwinTrackAdv::createRoute(Destination start, float fDx, float fDy, Destination _dest)
{
    // Cheating right here and not matching derivatives
    //x = bt + c
    //start = c
    // end = bt + c   end - c = b
    //DestinationEquation cheater(0, dest.fX - start.fX, start.fX, 0, dest.fY - start.fY, start.fY);
    //return cheater;

    Destination dest = rotateTranslateDestination(_dest);
    DestinationEquation straightShot(0, dest.fX - start.fX, start.fX, 0, dest.fY - start.fY, start.fY);
    return straightShot;
}

bool Drive_TwinTrackAdv::isRouteObstructed(DestinationEquation destEq, std::vector<Obstruction>& arObst, float fOffsetX, float fOffsetY)
{
    for (int i = 0; i < arObst.size(); i++) {
        if (arObst[i].IsObstructed(destEq, destEq.fT, fOffsetX, fOffsetY)) {
            return true;
        }
    }
    return false;
}

void Drive_TwinTrackAdv::buildRouteTree(Destination _dest, std::vector<Obstruction>& arObst)
{
    std::vector<Vertex*> vertList;
    VertexList leafList;
    std::vector<Vertex*> solutionList;
    // We rotate the destination about the origin so that we always start out at (0,0)
    //  with derivatives (direction) of (0,1)
    float fDx = 0;
    float fDy = 1;
    Destination dest = rotateTranslateDestination(_dest);
    // For the parent vertex, leave the coeff all 0, set t = 1
    DestinationEquation start;
    start.fT = 1;
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

   // Clear the current route list
    while (arRouteList.size() > 0) {
        delete arRouteList[arRouteList.size() - 1];
        arRouteList.pop_back();
    }
    // Now take the solution, and add it and all its parents to arRouteList. Set each entry
    //  in vertList to nullptr
    pCurrentRoute = solutionList[iShortestIndex];
    for (int i = 0; pCurrentRoute->GetParent() != nullptr; i++) {
        arRouteList[i] = pCurrentRoute;
        pCurrentRoute = pCurrentRoute->GetParent();
        if (pCurrentRoute == &vertParent) {
            break; // I think this is where it should end, need to verify this
        }
        for (int j = 0; j < vertList.size(); j++) {
            if (vertList[j] == pCurrentRoute) {
                vertList[j] = nullptr;
                break;
            }
        }
    }
    arRouteList.shrink_to_fit();
    // Now delete anything left in vertList
    for (int i = 0; i < vertList.size(); i++) {
        if (vertList[i] != nullptr) {
            delete vertList[i];
        }
    }
    // List is backwards
    for (int a = 0, z = arRouteList.size(); a < z; a++, z--) {
        Vertex* pTemp = arRouteList[a];
        arRouteList[a] = arRouteList[z];
        arRouteList[z] = pTemp;
    }
    pCurrentRoute = arRouteList[0];
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

Destination Drive_TwinTrackAdv::rotateTranslateDestination(Destination destFinal) const
{
    Destination destStart = pCurrentRoute->GetRoute().GetLocation(pCurrentRoute->GetRoute().fT);
    // First, translate destination so that our start is (0,0)
    destFinal.fX -= destStart.fX;
    destFinal.fY -= destStart.fY;
    // Now get derivative (direction) so we can rotate the destination about the start. Basically
    //  easier if we just line things up so we're always going straight up.
    float fDx;
    float fDy;
    pCurrentRoute->GetRoute().GetDeriv(fDx, fDy, pCurrentRoute->GetRoute().fT);
    // First check along axes
    float fAngle;
    if (fDx == 0) {
        fAngle = fDy > 0 ? 0 : 3.14;
    } else if (fDy == 0) {
        if (fDx > 0) {
            fAngle = -3.14/2;
        } else {
            fAngle = 3.14/2;
        }
    } else {
        fAngle = atan2(fDy, fDx); // NOT SURE THIS IS POINTED RIGHT
    }
    /*
[x, y, 1][cos  sin  0]
         [-sin cos  0]  =  [x*cos + y*-sin + 0*1, x*sin + y*cos + 0*1, ...]
         [0    0    1]
    */
    const float fSin = sin(fAngle);
    const float fCos = cos(fAngle);
    Destination destRotated;
    destRotated.fX = (destFinal.fX * fCos) - (destFinal.fY * fSin);
    destRotated.fY = (destFinal.fX * fSin) + (destFinal.fY * fCos);
    return destRotated;
}

bool Drive_TwinTrackAdv::processDestRunningAvg(Destination dest)
{
    arDestAvg[iDestAvgCursor] = dest;
    float fX = 0;
    float fY = 0;
    float fZ = 0;
    //SIMD candidate
    for (int i = 0; i < arDestAvg.size(); i++) {
        fX += arDestAvg[i].fX;
        fY += arDestAvg[i].fY;
        fZ += arDestAvg[i].fZ;
    }
    fX /= arDestAvg.size();
    fY /= arDestAvg.size();
    fZ /= arDestAvg.size();
    if (++iDestAvgCursor == arDestAvg.size()) iDestAvgCursor = 0;

    Destination destCurrent = arRouteList[arRouteList.size() - 1]->GetRoute().GetLocation(1.0);
    float fDistance = sqrt(((fX - destCurrent.fX) * (fX - destCurrent.fX)) +
                           ((fY - destCurrent.fY) * (fY - destCurrent.fY)) +
                           ((fZ - destCurrent.fZ) * (fZ - destCurrent.fZ)));
    if (fDistance > fDistBeforeChangeDest) {
        destAvg.fX = fX;
        destAvg.fY = fY;
        destAvg.fZ = fZ;
        return true;
    }
    return false;
}
