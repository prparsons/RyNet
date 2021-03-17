#include "serverDroneStatus.h"

#include <stdio.h>

ServerLoop::ServerLoop()
: mtData(mt)
{
    printf("ServerLoop\n");
}

void ServerLoop::run(void)
{
    printf("ServerLoop Thread\n");
}

void SetImage(image& im)
{

}

void ServerLoop::SetMaxDriveValues(float fMaxSteer, float fMaxSpeed, float fMaxClimb)
{

}

void ServerLoop::SetSpeed(float fSpeed)
{

}

void ServerLoop::SetSteer(float fSteer)
{

}

void ServerLoop::SetClimb(float fClimb)
{

}

void ServerLoop::SetOrientation(Orientation orient)
{

}

void ServerLoop::SetEquation(DestinationEquation eq)
{

}

void ServerLoop::SetObstructions(const std::vector<Obstruction>& arObstructions)
{

}

void ServerLoop::updateGPU()
{

}
