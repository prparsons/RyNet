#include <jetson-inference/segNet.h>
#include <jetson-inference/detectNet.h>
#include <jetson-utils/loadImage.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <signal.h>

#include <string.h>
#include <thread>

#include "navigation.h"
#include "dataLogger.h"

enum OptionArgument {
    SIMULATE_VIDEO = 0,
    UDP_SERVER_ADDRESS,
    NAV_TARGET,
    IMG_SLICES,
    SEGMENT_NET,
    DETECT_NET,
    OPTION_NONE
};

// Command line help (display and explain options)
void printHelp();
// Failure cleanup
void cleanupFailure();
void handle_interrupt(int iSigNum);
void handle_segfault(int iSigNum);
// If we want to stop the main application with a Ctrl+c signal, then we need to stop the camera and the Navigator.
// Simplest way I can think of at the moment is signal for depth to stop, let the main thread finishing joining
// it, then we'll have a pointer to bStopNavOnDepthFail that the interrupt can set to True.
// Or, if we used --continue then need to signal nav to stop
InterruptStop* pSignal_interrupt = nullptr;
bool* pSignal_bStopNavOnDepthFail = nullptr;

int main( int argc, char** argv )
{
    // Handle signals
    signal(SIGINT, handle_interrupt);
    signal(SIGSEGV, handle_segfault);

    // Bools for the various command line options
    bool bOptionArgument = false;
    bool bStopNavOnDepthFail = true;
    pSignal_bStopNavOnDepthFail = &bStopNavOnDepthFail;
    bool bSendRTL = false;
    bool bStartPointLogger = false;
    bool bSimulateVideo = false;
    bool bManualServerAddress = false;
    bool bRunRGB = true;
    bool bRunDepth = true;
    bool bRunAccel = false;
    bool bDownsampleDepth = false;
    // Command line arguments
    OptionArgument OptionArgumentSelection = OPTION_NONE;
    char scVideoSimPath[512];
    scVideoSimPath[0] = 0;
    char scServerAddress[32];
    int iTargetClass = -1;
    int iNumSlices = -1;
    // Read all the command line arguments and options (--option argument)
    for (int i = 1; i < argc; i++) {
        if (bOptionArgument) {
            // For now, only thing is read possible video filename
            bOptionArgument = false;
            switch (OptionArgumentSelection) {
                case SIMULATE_VIDEO: strncpy(scVideoSimPath, argv[i], 512); break;
                case UDP_SERVER_ADDRESS: strncpy(scServerAddress, argv[i], 32); break;
                case DETECT_NET: {
                    if (!ObjectDetection::SetNetwork(argv[i])) {
                        printL("Invalid Detection network specified");
                        return -1;
                    }
                } break;
                case SEGMENT_NET: {
                    if (!TerrainSegmentation::SetNetwork(argv[i])) {
                        printL("Invalid Segmentation network specified");
                        return -1;
                    }
                } break;
                case NAV_TARGET: {
                    iTargetClass = atoi(argv[i]);
                    if (iTargetClass < 0 || iTargetClass > 255) {
                        printL("Target Class must be number between 0 and 255");
                        return -1;
                    }
                } break;
                case IMG_SLICES: {
                    iNumSlices = atoi(argv[i]);
                    if (iNumSlices < 0 || iNumSlices > 9) {
                        printL("Number of slices must be number between 0 and 9");
                        return -1;
                    }
                } break;
                default:
                    printL("OptionArgumentSelection Invalid");
                    return -1;
            }
            OptionArgumentSelection = OPTION_NONE;
        } else {
            if (strcmp("--continue", argv[i]) == 0) bStopNavOnDepthFail = false;
            else if (strcmp("--rtlfinish", argv[i]) == 0) bSendRTL = true;
            else if (strcmp("--no_rgb", argv[i]) == 0) bRunRGB = false;
            else if (strcmp("--no_depth", argv[i]) == 0) bRunDepth = false;
            else if (strcmp("--log_visited", argv[i]) == 0) bStartPointLogger = true;
            else if (strcmp("--downsample_depth", argv[i]) == 0) bDownsampleDepth = true;
            else if (strcmp("--run_accel", argv[i]) == 0) bRunAccel = true;
            else if (strcmp("--detection_net", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = DETECT_NET;
            } else if (strcmp("--segment_net", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = SEGMENT_NET;
            } else if (strcmp("--simulate_video", argv[i]) == 0) {
                bSimulateVideo = true;
                bOptionArgument = true;
                OptionArgumentSelection = SIMULATE_VIDEO;
            } else if (strcmp("--server_address", argv[i]) == 0) {
                bManualServerAddress = true;
                bOptionArgument = true;
                OptionArgumentSelection = UDP_SERVER_ADDRESS;
            } else if (strcmp("--nav_target", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = NAV_TARGET;
            } else if (strcmp("--img_slices", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = IMG_SLICES;
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
        if (bRunAccel) uCameraFeatures += CameraController_Accel;
        if (bDownsampleDepth) uCameraFeatures += CameraController_DownsampleDepth;
        if (bSimulateVideo) pCamera = CameraController::OpenCamera(uCameraFeatures, scVideoSimPath);
        else pCamera = CameraController::OpenCamera(uCameraFeatures);

        printL(">>>Getting Flight Controller");
        FlightController* pController = FlightController::GetFlightController(logger, *pCamera);
        if (bManualServerAddress) pController->SetServerAddress(scServerAddress);

        printL(">>>Creating Navigator");
        Navigator nav(*pController, logger, *pCamera);
        if (bRunDepth) pSignal_interrupt = pCamera;
        else pSignal_interrupt = &nav;
        // If we specified a target class, need to set that now
        if (iTargetClass != -1) nav.SetTargetClass(uint8_t(iTargetClass));
        if (iNumSlices != -1) nav.SetNumberImageSlices(iNumSlices);

        printL(">>>Starting DataLogger");
        logger.Start();
        printL(">>>Starting Camera");
        pCamera->Start(); // Camera starts all it's own threads
        printL(">>>Starting Navigator");
        nav.Start();
        printL(">>>Starting Flight Controller");
        pController->Start(bRunDepth, bStartPointLogger);

        // std::system_error if the thread could not be started.
        //**I think there might be issues with shutdown of stuff...need to trace through it all and double check
        printL(">>>Threads Started, waiting on camera.join()");        
        pCamera->JoinDepth();
        printL(">>>Camera Depth joined");
        // If we get here, depth failed 
        if (bStopNavOnDepthFail) {
            printL(">>>Sending stop signal to stop Nav");
            nav.Stop();
            ///***this has changed a bit since -> //**I got a seg fault once without --continue, I wonder if it's something to do with this? (datalogger wasn't cleaning up threads...)
        } else {
            nav.Join();
        }// else could have a set alititude option, climb a bit if no depth?
        printL(">>>Navigator Stopped");
        if (bSendRTL) pController->SetRTL();
        logger.Stop();
        printL(">>>Datalogger Stopped");
        pController->DestroyFlightController();
        pController = nullptr;
        printL(">>>Flight Controller destroyed");
        delete pCamera;
    } catch (std::exception e) {
        std::cout << "Exception: " << e.what() << std::endl;
        std::cout << "Sending RTL";
        cleanupFailure();
    }
    return 0;
}

void printHelp()
{
    printL("Environment Variables");
    printL(" SERVER_ADDRESS");
    printL("    In the event of failure (seg fault, unhandled exception, etc.) an RTL will");
    printL("    be sent to this address. If empty, 127.0.0.1 will be used instead.\n\n");
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
    printL(" --downsample_depth");
    printL("    Enable decimation filter that will downsample the depth frames.\n");
    printL(" --run_accel");
    printL("    Enable the depth camera's accelerometer.\n");
    printL(" --nav_target [XX]");
    printL("    Manually specify the target class the Navigation network should target.");
    printL("    Target is number from 0-255.\n");
    printL(" --img_slices [XX]");
    printL("    Number of horizontal slices of the RGB image before it is used for");
    printL("    detection and segmentation. Target is number from 0-9; default is 3.\n");
    printL(" --detection_net [ABCDE]");
    printL("    Network model to be used with object detection network. Valid choices are");
    printL("    \"SSD_MOBILENET_V1\", \"SSD_MOBILENET_V2\", \"SSD_INCEPTION_V2\".\n");
    printL(" --segment_net [ABCDE]");
    printL("    Network model to be used with the image segmentation network. segnet --help");
    printL("    will list available models.\n");
}

void cleanupFailure()
{
    const char* scServerAddress = getenv("SERVER_ADDRESS");
    if (scServerAddress == nullptr) scServerAddress = "127.0.0.1";
    // What if we exec a script that just sends a RTL to the server script?
    struct sockaddr_in addressMavLink;
    int socketDesc;
    socketDesc = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketDesc < 0) return;
    addressMavLink.sin_family = AF_INET; 
    addressMavLink.sin_port = htons(3636); 
    addressMavLink.sin_addr.s_addr = inet_addr(scServerAddress);
    sendto(socketDesc, "<rtl />", 7, MSG_CONFIRM, (const struct sockaddr*) &addressMavLink, sizeof(addressMavLink));
}

void handle_interrupt(int iSigNum)
{
    printL("Handling Interrupt");
    if (pSignal_bStopNavOnDepthFail == nullptr) {
        printL("interrupt bStopNavOnDepthFail not set, unable to close program gracefully.");
        exit(iSigNum);
    }
    if (pSignal_interrupt == nullptr) {
        printL("interrupt pCamera not set, unable to close program gracefully.");
        exit(iSigNum);
    }
    *pSignal_bStopNavOnDepthFail = true;
    pSignal_interrupt->SignalStop();
}

void handle_segfault(int iSigNum)
{
    printL("Handling Segmentation Fault");
    cleanupFailure();
    exit(iSigNum);
}
