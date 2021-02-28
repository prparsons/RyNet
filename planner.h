#pragma once

#include "navigation.h"

class Vertex;

class DrivePlanner {
public:
    DrivePlanner(std::vector<Navigator*>& arNav);
    virtual ~DrivePlanner() {}

    virtual bool isPlanValid(Navigator& nav, Navigator::dirty isDirty) = 0;
    virtual void createNewPlan() = 0;

    float getSpeed() const { return fSpeed; }
    float getSteer() const { return fSteer; }
    float getClimb() const { return fClimb; }
    Orientation getOrient() const { return orient; }

    virtual int getLoopTimeoutMS() const { return 1000;}

protected:
    std::vector<Navigator*>& arNav;
    DestinationEquation* pDesFinal; // Destination (pointer to bottom of route list)
    std::vector<DestinationEquation> arRoute; // List of routes to get to destination

    float fSpeed; // 0 <-> 1 stopped to max speed
    float fSteer; // -1 <-> 1 full left to full right
    float fClimb; // -1 <-> 1 full dive to full climb
    float fMaxSteer; // Max steering in degrees per sec
    float fMaxSpeed; // m/s
    float fMaxClimb; // m/s
    // Direction of travel
    Orientation orient;
    
    //Need mapping from equation t to realtime
    //Need way to get new speed settings while incrementing t the correct amount since last update (time() libraries?)
};

class Drive_TwinTrack: public DrivePlanner {
public:
    Drive_TwinTrack(std::vector<Navigator*>& arNav);

    virtual bool isPlanValid(Navigator& nav, Navigator::dirty isDirty) override;
    virtual void createNewPlan() override;

private:
    std::vector<float> arSteerAvg;
};

class Drive_TwinTrackAdv: public DrivePlanner {
public:
    Drive_TwinTrackAdv(std::vector<Navigator*>& arNav);

    virtual bool isPlanValid(Navigator& nav, Navigator::dirty isDirty) override;
    virtual void createNewPlan() override;

protected:
    DestinationEquation createRoute(Destination start, float fDx, float fDy, Destination dest);
    bool isRouteObstructed(DestinationEquation, std::vector<Obstruction>&);

    void buildRouteTree(Destination dest, std::vector<Obstruction>& arObst);
    Vertex** branch(Vertex& vert);

    Vertex* pSolution;
    std::vector<Vertex*> arSolutionParentList;
    Vertex* pCurrentSpot;
    float fT;

    std::vector<Destination> arDestAvg;
    std::vector<float> arSpeedAvg;
    std::vector<Orientation> arOrientAvg;
};
