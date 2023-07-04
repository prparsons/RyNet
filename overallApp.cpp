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
    UDP_KEY,
    WAIT_TAKEOFF,
    GEO_DIST,
    UDP_TIMEOUT,
    GUIDE_TIME,
    GUIDE_SPEED,
    GUIDE_PERCENT,
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
bool* pSignal_bAbortTakeoff = nullptr;

int main( int argc, char** argv )
{
    // Handle signals
    signal(SIGINT, handle_interrupt);
    signal(SIGSEGV, handle_segfault);

    // Bools for the various command line options
    bool bOptionArgument = false;
    bool bBatteryMonitor = false;
    bool bStopNavOnDepthFail = true;
    pSignal_bStopNavOnDepthFail = &bStopNavOnDepthFail;
    bool bSendRTL = false;
    bool bStartPointLogger = false;
    bool bSimulateVideo = false;
    bool bManualServerAddress = false;
    bool bRunRGB = true;
    bool bRunDepth = true;
    bool bRunAccel = false;
    bool bRunFlir = false;
    bool bDownsampleDepth = false;
    bool bHmacUdp = false;
    // Command line arguments
    OptionArgument OptionArgumentSelection = OPTION_NONE;
    char scVideoSimPath[512];
    scVideoSimPath[0] = 0;
    char scServerAddress[32];
    char scUdpKey[33];
    int iTargetClass = -1;
    int iNumSlices = -1;
    int iTakeoffAltitude = -1;
    int iGeoDist = -1;
    int iUdpTimeoutUs = -1;
    int iGuideSpeed = -1;
    int iGuideTime = -1;
    float fGuideExtraPercent = 0;
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
                case UDP_KEY: {
                    if (strlen(argv[i]) > 32) {
                        printL("Key is too long, it must not be more than 32 characters");
                        return -1;
                    }
                    bHmacUdp = true;
                    strncpy(scUdpKey, argv[i], 32);
                } break;
                case GEO_DIST: {
                    iGeoDist = atoi(argv[i]);
                    if (iGeoDist < 1 || iGeoDist > 9999) {
                        printL("Maximum distance from takeoff must be between 1 and 9999.");
                        return -1;
                    }
                } break;
                case UDP_TIMEOUT: {
                    iUdpTimeoutUs = atoi(argv[i]);
                    if (iUdpTimeoutUs < 1 || iUdpTimeoutUs > 1000000) {
                        printL("UDP timeout must be between 1-1000000 microseconds.");
                        return -1;
                    }
                } break;
                case GUIDE_SPEED: {
                    iGuideSpeed = atoi(argv[i]);
                    if (iGuideSpeed < 1 || iGuideSpeed > 99) {
                        printL("Guided speed must be between 1 and 99.");
                        return -1;
                    }
                } break;
                case GUIDE_TIME: {
                    iGuideTime = atoi(argv[i]);
                    if (iGuideTime < 1 || iGuideTime > 60) {
                        printL("Guided time between changes must be between 1 and 60.");
                        return -1;
                    }
                } break;
                case GUIDE_PERCENT: {
                    fGuideExtraPercent = atoi(argv[i]);
                    if (fGuideExtraPercent < 1.0 || fGuideExtraPercent > 2.0) {
                        printL("Guided extra percent must be between 1.0 and 2.0.");
                        return -1;
                    }
                }
                case WAIT_TAKEOFF: {
                    iTakeoffAltitude = atoi(argv[i]);
                    if (iTakeoffAltitude < 10 || iTakeoffAltitude > 120) {
                        printL("Takeoff wait altitude must be between 10 and 120");
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
            else if (strcmp("--battery_monitor", argv[i]) == 0) bBatteryMonitor = true;
            else if (strcmp("--run_flir", argv[i]) == 0) bRunFlir = true;
            else if (strcmp("--hmac_udp", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = UDP_KEY;
            } else if (strcmp("--detection_net", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = DETECT_NET;
            } else if (strcmp("--max_dist", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = GEO_DIST;
            } else if (strcmp("--guide_time", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = GUIDE_TIME;
            } else if (strcmp("--guide_speed", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = GUIDE_SPEED;
            } else if (strcmp("--guide_percent", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = GUIDE_PERCENT;
            } else if (strcmp("--wait_takeoff", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = WAIT_TAKEOFF;
            } else if (strcmp("--udp_timeout", argv[i]) == 0) {
                bOptionArgument = true;
                OptionArgumentSelection = UDP_TIMEOUT;
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
        if (bRunFlir) uCameraFeatures += CameraController_IR;
        if (bSimulateVideo) pCamera = CameraController::OpenCamera(uCameraFeatures, scVideoSimPath);
        else pCamera = CameraController::OpenCamera(uCameraFeatures);

        printL(">>>Getting Flight Controller");
        FlightController* pController = FlightController::GetFlightController(logger, *pCamera);
        if (bManualServerAddress) pController->SetServerAddress(scServerAddress);
        if (bHmacUdp) pController->SetUdpKey(scUdpKey);
        if (iUdpTimeoutUs != -1) pController->SetUdpTimout(iUdpTimeoutUs);
        if (iGuideSpeed != -1) pController->SetMissionSpeed(iGuideSpeed);
        if (fGuideExtraPercent != 0.0) pController->SetPercentExtraDistance(fGuideExtraPercent);

        printL(">>>Creating Navigator");
        Navigator nav(*pController, logger, *pCamera, bBatteryMonitor);
        if (bRunDepth) pSignal_interrupt = pCamera;
        else pSignal_interrupt = &nav;
        // If we specified a target class, need to set that now
        if (iTargetClass != -1) nav.SetTargetClass(uint8_t(iTargetClass));
        if (iNumSlices != -1) nav.SetNumberImageSlices(iNumSlices);
        if (iGuideTime != -1) nav.SetTimeBetweenChanges(iGuideTime);

        printL(">>>Starting DataLogger");
        logger.Start();
        printL(">>>Starting Camera");
        pCamera->Start(); // Camera starts all it's own threads
        printL(">>>Starting Flight Controller");
        if (iGeoDist != -1) pController->Start(bRunDepth, bStartPointLogger, bRunFlir, iGeoDist);
        else pController->Start(bRunDepth, bStartPointLogger, bRunFlir);
        if (iTakeoffAltitude != -1) {
            bool bAbort = false;
            pSignal_bAbortTakeoff = &bAbort;
            pController->WaitForTakeoff(iTakeoffAltitude, bAbort);
            pSignal_bAbortTakeoff = nullptr;
        }
        printL(">>>Starting Navigator");
        nav.Start();

        // std::system_error if the thread could not be started.
        //**I think there might be issues with shutdown of stuff...need to trace through it all and double check
        printL(">>>Threads Started, waiting on camera.join()");        
        pCamera->JoinDepth();
        printL(">>>Camera Depth joined");
        // If we get here, depth failed 
        if (bStopNavOnDepthFail) {
            printL(">>>Sending stop signal to stop Nav");
            nav.SignalStop();
            ///***this has changed a bit since -> //**I got a seg fault once without --continue, I wonder if it's something to do with this? (datalogger wasn't cleaning up threads...)
        } else {
            // If depth was started but failed, and continue also specified...then currently we're about to wait on
            //  nav.join, but pSignal_interrupt is set to the depth camera. So change it to nav so it will be stopped 
            //  by the interrupt
            printL(">>>Changing interrupt signal to nav");
            pSignal_interrupt = &nav;
        }

        nav.Join();
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
    printL(" --battery_monitor");
    printL("    Run battery monitor thread that will stop navigation and send RTL if");
    printL("    battery level drops below 50%.\n");
    printL(" --run_flir");
    printL("    Enable the flir thermal camera on /dev/video4.\n");
    printL(" --nav_target [XX]");
    printL("    Manually specify the target class the Navigation network should target.");
    printL("    Target is number from 0-255.\n");
    printL(" --img_slices [XX]");
    printL("    Number of horizontal slices of the RGB image before it is used for");
    printL("    detection and segmentation.");
    printL("    Target is number from 0-9; default is 3.\n");
    printL(" --detection_net [ABCDE]");
    printL("    Network model to be used with object detection network. Valid choices are");
    printL("    \"SSD_MOBILENET_V1\", \"SSD_MOBILENET_V2\", \"SSD_INCEPTION_V2\".\n");
    printL(" --segment_net [ABCDE]");
    printL("    Network model to be used with the image segmentation network. segnet --help");
    printL("    will list available models.\n");
    printL(" --hmac_udp [ABCDE]");
    printL("    All UDP messages sent/received will be verified with a salted hmac.");
    printL("    Target is a string up to 32 characters long with no spaces.\n");
    printL(" --wait_takeoff [XX]");
    printL("    Altitude drone must reach before navigation routine starts.");
    printL("    Target is number from 10-120m.\n");
    printL(" --max_dist [XX]");
    printL("    Maximum distance allowed from takeoff position.");
    printL("    Target is number from 1-9999.\n");
    printL(" --udp_timeout [XX]");
    printL("    Timeout value for UDP packets in microseconds.");
    printL("    Target is number from 1-1000000.\n");
    printL(" --guide_speed [XX]");
    printL("    Speed in meters per second to use when calculating points in guided mode.");
    printL("    Target is number from 1-99. Default is read from Mission List.\n");
    printL(" --guide_percent [XX]");
    printL("    To calculate distance for a guided point mission, waypoint speed and");
    printL("    guide_time is used to calculate. Then this percentage is applied to that");
    printL("    distance so the drone doesn't start slowing down (and pitching up)");
    printL("    before having the chance to calculate new point or resume AUTO");
    printL("    Target is number >= 1.0 and <= 2.0. Default is 1.10.\n");
    printL(" --guide_time [XX]");
    printL("    Time in seconds between allowed changes in mission. This time must pass");
    printL("    before mode will be changed to/from AUTO, or before a new GUIDED point");
    printL("    will be entered.");
    printL("    Target is number from 1-60.\n");
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
    if (pSignal_bAbortTakeoff != nullptr) {
        *pSignal_bAbortTakeoff = true;
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
