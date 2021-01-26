#include "serverDroneStatus.h"

#include <stdio.h>

ServerLoop::ServerLoop()
{
    printf("ServerLoop\n");
}

void ServerLoop::operator()(void)
{
    printf("ServerLoop Thread\n");
}
