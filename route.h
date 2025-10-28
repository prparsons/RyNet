#pragma once

#include <string>

class CoordNode {
    public:
        CoordNode(float fLat, float fLon);
        CoordNode(CoordNode& other, float fLat, float fLon); // Theft Constructor
        ~CoordNode();
    
        float GetDistance(float fLatQuery, float fLonQuery) const;
        const CoordNode* GetClosest(float fLatQuery, float fLonQuery) const;
        const CoordNode* GetClosest(float fLatQuery, float fLonQuery, float& fDistance) const;
        void InsertPoint(float fLatNew, float fLonNew);
        void ResetPosition(float fLat, float fLon); // Clears list and reset position
    
        void DestroyChildren();
        void Print() const;
    
    private:
        float fLat;
        float fLon;
    
        CoordNode* nw;
        CoordNode* ne;
        CoordNode* se;
        CoordNode* sw;
    };

class Route
{
public:
    struct command {
        int16_t seq;
        int16_t current;
        int16_t frame;
        int16_t command; // 16 is a waypoint
        float param1;
        float param2;
        float param3;
        float param4;
        float lat;
        float lon;
        float alt;
        int16_t autocontinue;
    };

    Route(const Route& other);
    ~Route();

    std::string toString() const;
    float GetDistance() const;

    static Route* CreateRoute(char* buffer, size_t uBufferLen);
    static Route* CleanRoute(Route* pStart, CoordNode* pVisitedPoints, int16_t iNextPoint, bool b2opt);

    command cmd;

private:
    Route(char* buffer, Route& prev);
    Route(char* buffer, size_t uBufferLen);
    Route(const command& cmdCopy);

    static Route* cleanRoute(Route* pStart, CoordNode* pVisitedPoints); // Takes over pStart, returns new start
    static Route* removeVisited(Route* pStart, int16_t iNextPoint); // Takes over pStart, returns new start
    static Route* single2Opt(const Route& start); 

    Route* pNext;
};
