#pragma once

#include <string>

class CoordNode {
    public:
        CoordNode(float fLat, float fLon);
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
        int16_t command;
        float param1;
        float param2;
        float param3;
        float param4;
        float lat;
        float lon;
        float alt;
        int16_t autocontinue;
    };
    Route(char* buffer, Route& prev);
    Route(char* buffer, size_t uBufferLen);
    ~Route();
    std::string toString() const;
    Route* twoOpt();

    command cmd;
    Route* pNext;
};