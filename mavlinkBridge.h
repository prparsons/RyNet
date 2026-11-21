#pragma once

#include <unistd.h>

class MavLinkBridge {
public:
    MavLinkBridge();

    bool LaunchBridge(int iPort, bool bTest, const char* scUdp = nullptr);
    void StopBridge();
    
private:
    pid_t pid;
};