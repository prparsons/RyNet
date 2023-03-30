#include <jetson-inference/segNet.h>
#include <jetson-inference/detectNet.h>
#include <jetson-utils/loadImage.h>
#include <sys/socket.h>

#include <string.h>
#include <thread>

#include "navigation.h"
#include "dataLogger.h"

enum OptionArgument {
    SIMULATE_VIDEO = 0,
    UDP_SERVER_ADDRESS,
    OPTION_NONE
};

// Command line help (display and explain options)
void printHelp();

int main( int argc, char** argv )
{
    // Bools for the various command line options
    bool bOptionArgument = false;
    bool bStopNavOnDepthFail = true;
    bool bSendRTL = false;
    bool bStartPointLogger = false;
    bool bSimulateVideo = false;
    bool bManualServerAddress = false;
    bool bRunRGB = true;
    bool bRunDepth = true;
    // Command line arguments
    OptionArgument OptionArgumentSelection = OPTION_NONE;
    char scVideoSimPath[512];
    scVideoSimPath[0] = 0;
    char scServerAddress[32];
    // Read all the command line arguments and options (--option argument)
    for (int i = 1; i < argc; i++) {
        if (bOptionArgument) {
            // For now, only thing is read possible video filename
            bOptionArgument = false;
            switch (OptionArgumentSelection) {
                case SIMULATE_VIDEO: strncpy(scVideoSimPath, argv[i], 512); break;
                case UDP_SERVER_ADDRESS: strncpy(scServerAddress, argv[i], 32); break;
                default:
                    printL("OptionAgurmentSelection Invalid");
                    return -1;
            }
            OptionArgumentSelection = OPTION_NONE;
        } else {
            if (strcmp("--continue", argv[i]) == 0) bStopNavOnDepthFail = false;
            else if (strcmp("--rtlfinish", argv[i]) == 0) bSendRTL = true;
            else if (strcmp("--no_rgb", argv[i]) == 0) bRunRGB = false;
            else if (strcmp("--no_depth", argv[i]) == 0) bRunDepth = false;
            else if (strcmp("--log_visited", argv[i]) == 0) bStartPointLogger = true;
            else if (strcmp("--simulate_video", argv[i]) == 0) {
                bSimulateVideo = true;
                bOptionArgument = true;
                OptionArgumentSelection = SIMULATE_VIDEO;
            } else if (strcmp("--server_address", argv[i]) == 0) {
                bManualServerAddress = true;
                bOptionArgument = true;
                OptionArgumentSelection = UDP_SERVER_ADDRESS;
            } else {
                if (strcmp("--help", argv[i]) != 0) {
                    printf("Invalid option [%s]\n", argv[i]);
                }
                printHelp();
                return 0;
            }
        }
    }
    try {
        DataLogger logger;

        CameraController* pCamera;
        printL(">>>Opening Camera");
        uint32_t uCameraFeatures = 0;
        if (bSimulateVideo) uCameraFeatures += CameraController_RGBSimulation;
        else if (bRunRGB) uCameraFeatures += CameraController_RGB;
        if (bRunDepth) uCameraFeatures += CameraController_Depth;
        pCamera = CameraController::OpenCamera(uCameraFeatures, scVideoSimPath);

        printL(">>>Getting Flight Controller");
        FlightController* pController = FlightController::GetFlightController(logger, *pCamera);
        if (bManualServerAddress) pController->SetServerAddress(scServerAddress);

        printL(">>>Creating Navigator");
        Navigator nav(*pController, logger, *pCamera);

        printL(">>>Starting DataLogger");
        std::thread thrLogger(&DataLogger::run, &logger);
        printL(">>>Starting Camera");
        pCamera->run(); // Camera starts all it's own threads
        printL(">>>Starting Navigator");
        std::thread thrNav(&Navigator::runNavigator, &nav);
        pController->Start(bRunDepth, bStartPointLogger);

        // std::system_error if the thread could not be started.
        //**I think there might be issues with shutdown of stuff...need to trace through it all and double check
        printL(">>>Threads Started, waiting on camera.join()");        
        pCamera->JoinDepth();
        printL("Camera Depth joined");
        // If we get here, depth failed 
        if (bStopNavOnDepthFail) {
            printL("Getting stop signal to stop Nav");
            StopSignal* pss = nav.GetStopSignal();
            pss->stop();
            delete pss; //**I got a seg fault once without --continue, I wonder if it's something to do with this?
        } // else could have a set alititude option, climb a bit if no depth?
        thrNav.join();
        printL(">>>Navigator Stopped");
        if (bSendRTL) pController->SetRTL();
        thrLogger.join();
        printL(">>>Datalogger Stopped");
        pController->DestroyFlightController();
        pController = nullptr;
        delete pCamera;
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

void printHelp()
{
    printL("Command line options:");
    printL(" --continue");
    printL("    Program will not stop if the depth camera fails. Otherwise, if the depth");
    printL("    camera fails, navigation will stop and the program will exit.\n");
    printL(" --rtlfinish");
    printL("    Drone will ReturnToLaunch after finishing navigation. Otherwise, it will");
    printL("    only execute the last mission command.\n");
    printL(" --simulate_video [VIDEO_FILE]");
    printL("    Program will open the specified VIDEO_FILE and use that imagery instead");
    printL("    starting the RGB camera. Failure to open video file will cause an ");
    printL("    exception.\n");
    printL(" --server_address [XXX.XXX.XXX.XXX]");
    printL("    Specified ip address will be used as the Mavlink UDP server. If not");
    printL("    provided, 127.0.0.1 will be used as a default.\n");
    printL(" --no_rgb");
    printL("    Do not start the RGB camera.\n");
    printL(" --no_depth");
    printL("    Do not start the depth camera.\n");
    printL(" --log_visited");
    printL("    Will log visited points.\n");
}
