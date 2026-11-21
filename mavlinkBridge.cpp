#include <stdio.h>
#include <string.h>
#include <sys/wait.h>

#include "mavlinkBridge.h"

MavLinkBridge::MavLinkBridge()
: pid(0)
{

}

bool MavLinkBridge::LaunchBridge(const char* scConnect, int iPort, bool bTest, const char* scUdp)
{
    // Set up arguments
    char scConnectLocal[] = "/dev/ttyTHS0";
    if (strcmp(scConnect, "local") == 0) {
        scConnect = scConnectLocal;
    }
    char scPort[256];
    snprintf(scPort, 256, "%d", iPort);
    const char* args[10];
    size_t uIdx = 0;
    args[uIdx++] = "python3";
    args[uIdx++] = "rynetMavServe.py";
    args[uIdx++] = "--connect";
    args[uIdx++] = scConnect;
    args[uIdx++] = "--port";
    args[uIdx++] = scPort;
    if (scUdp) {
        args[uIdx++] = "--udp_hmac";
        args[uIdx++] = scUdp;
    }
    if (bTest) args[uIdx++] = "--liveTest";
    for (size_t u = uIdx; u < 10; u++) args[u] = nullptr;

    // Now start the new process
    pid = fork();
    if (pid < 0) {
        //todo fork failed
        return false;
    }
    if (pid == 0) {
        execvp("python3", (char* const *)args);
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