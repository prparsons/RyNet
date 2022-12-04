#include <jetson-inference/segNet.h>
#include <jetson-inference/detectNet.h>
#include <jetson-utils/loadImage.h>
#include <sys/socket.h>

#include <string.h>
#include <thread>

#include "navigation.h"
#include "dataLogger.h"

int main( int argc, char** argv )
{
    bool bStopNavOnDepthFail = true;
    bool bSendRTL = false;
    for (int i = 0; i < argc; i++) {
        if (strcmp("--continue", argv[i]) == 0) bStopNavOnDepthFail = false;
        else if (strcmp("--rtlfinish", argv[i])) bSendRTL = true;
    }
    try {
        DataLogger logger;
        CameraController* pCamera = CameraController::OpenCamera(CameraController_RGB | CameraController_Depth);
        FlightController controller(logger, *pCamera);
        Navigator nav(controller, logger, *pCamera);
        printL("Starting main loop");
        std::thread thrLogger(&DataLogger::run, &logger);
        pCamera->run(); // Camera starts all it's own threads
        std::thread thrNav(&Navigator::runNavigator, &nav);
        // std::system_error if the thread could not be started.
        printL("Threads Started, waiting on camera");        
        pCamera->JoinDepth();
        printL("Camera Depth joined");
        // If we get here, depth failed 
        if (bStopNavOnDepthFail) {
            printL("Getting stop signal to stop Nav");
            StopSignal* pss = nav.GetStopSignal();
            pss->stop();
            delete pss;
        } // else could have a set alititude option, climb a bit if no depth?
        thrNav.join();
        if (bSendRTL) controller.SetRTL();
        thrLogger.join();
    } catch (std::exception e) {
        std::cout << "Exception: " << e.what() << std::endl;
        std::cout << "Sending RTL";
        // What it we exec a script that just sends a RTL to the server script?
        struct sockaddr_in addressMavLink;
        int socketDesc;
        socketDesc = socket(AF_INET, SOCK_DGRAM, 0);
        if (socketDesc < 0) return 1;
        addressMavLink.sin_family = AF_INET; 
        addressMavLink.sin_port = htons(3636); 
        addressMavLink.sin_addr.s_addr = INADDR_ANY;
        sendto(socketDesc, "<rtl />", 7, MSG_CONFIRM, (const struct sockaddr*) &addressMavLink, sizeof(addressMavLink));
    }
    return 0;
}
