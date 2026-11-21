#include <stdio.h>
#include <sys/wait.h>

#include "mavlinkBridge.h"

MavLinkBridge::MavLinkBridge()
: pid(0)
{

}

bool MavLinkBridge::LaunchBridge(int iPort, bool bTest, const char* scUdp)
{
    // Set up arguments
    const char* scConnect = "/dev/ttyTHS0";
    char scPort[256];
    snprintf(scPort, 256, "%d", iPort);
    const char* args[8];
    args[0] = "rynetMavServe.py";
    args[1] = "--connect";
    args[2] = scConnect;
    args[3] = "--port";
    args[4] = scPort;
    size_t uIdx = 5;
    if (scUdp) {
        args[uIdx++] = "--udp_hmac";
        args[uIdx++] = scUdp;
    }
    if (bTest) args[uIdx++] = "--liveTest";
    for (size_t u = uIdx; u < 8; u++) args[u] = nullptr;

    // Now start the new process
    pid = fork();
    if (pid < 0) {
        //todo fork failed
        return false;
    }
    if (pid == 0) {
        execvp("/usr/bin/python3", (char* const *)args);
        //todo printf failed to execvp
        return false;
    }

    return true;
}

void MavLinkBridge::StopBridge()
{
    int iStopStatus = 0;
    if (pid > 0) {
        kill(pid, SIGTERM);
        waitpid(pid, &iStopStatus, 0);
    }
}