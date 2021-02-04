#pragma once

#include "navigation.h"

class RoutePlan {
public:
    Destination dest;
    DestinationEquation route;
};

class DrivePlanner {
public:
    DrivePlanner(std::vector<Navigator*>& arNav);
    virtual ~DrivePlanner() {}

    virtual bool _decide(Navigator& nav, Navigator::dirty isDirty) = 0;
    virtual bool _decide() = 0;

    float getSpeed() const { return fSpeed; }
    float getSteer() const { return fSteer; }
    float getClimb() const { return fClimb; }

    virtual int getLoopTimeoutMS() const { return 1000;}

protected:
    std::vector<Navigator*>& arNav;
    RoutePlan* pDesFinal; // Destination (pointer to bottom of route list)
    std::vector<RoutePlan> arRoute; // List of routes to get to destination

    float fSpeed; // 0 <-> 1 stopped to max speed
    float fSteer; // -1 <-> 1 full left to full right
    float fClimb; // -1 <-> 1 full dive to full climb
};

class Drive_TwinTrack: public DrivePlanner {
public:
    Drive_TwinTrack(std::vector<Navigator*>& arNav);
    virtual bool _decide(Navigator& nav, Navigator::dirty isDirty) override;
    virtual bool _decide() override;
};
