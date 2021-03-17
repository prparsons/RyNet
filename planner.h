#pragma once

#include "navigation.h"
#include "data_types.h"

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
    void createRoute(Destination start, float fDx, float fDy, Destination dest, std::vector<DestinationEquation>& arRoute);
    DestinationEquation createRoute(Destination start, float fDx, float fDy, Destination dest);
    bool isRouteObstructed(DestinationEquation, std::vector<Obstruction>& arObs, float fOffsetX = 0, float fOffsetY = 0);

    void buildRouteTree(Destination dest, std::vector<Obstruction>& arObst);
    Vertex** branch(Vertex& vert);
    Destination rotateTranslateDestination(Destination dest) const;

    bool processDestRunningAvg(Destination dest); // Returns true if change

    Vertex* pCurrentRoute;
    std::vector<Vertex*> arRouteList;

    std::vector<Destination> arDestAvg;
    std::vector<float> arSpeedAvg;
    std::vector<Orientation> arOrientAvg;
    Destination destAvg;
    float fDistBeforeChangeDest;
    int iDestAvgCursor;
    int iSpeedAvgCursor;
    int iOrientationAvgCursor;
};
