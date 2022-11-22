#include "planner.h"

#include <vector>
#include "comms.h"

FlightController::FlightController(DataLogger& _logger)
: logger(_logger)
, pThreadDepth(nullptr)
{

}

void FlightController::SetDirection(float fHeadingOffsetDeg)
{

}

void FlightController::SetDestination(float fHeadingOffsetDeg, float fDistance)
{

}

void FlightController::RestoreRouteNaive()
{

}

void FlightController::RestoreRouteClean()
{

}

void FlightController::SaveRoute()
{

}

void FlightController::runDepth()
{

}


void FlightController::SendXML(const char* strMessage)
{	
	//Send some data
	if(send(socketDesc, strMessage, strlen(strMessage), 0) < 0) {
		printL("SendXML fail, attempting to connect");
        ConnectMavLink();
        if (send(socketDesc, strMessage, strlen(strMessage), 0) < 0) {
            printL("  That didn't work, still failed");
        }
	}
}

char* FlightController::SendRecXML(const char* strMessage)
{	
	//Send some data
	if(send(socketDesc, strMessage, strlen(strMessage), 0) < 0) {
		printL("SendRecXML fail, attempting to connect");
        ConnectMavLink();
        if (send(socketDesc, strMessage, strlen(strMessage), 0) < 0) {
            printL("  That didn't work, still failed");
            return nullptr;
        }
	}
    char* scResponse = new char[512];
    if(recv(socketDesc, scResponse, sizeof(511), 0) < 0){
        printL("SendRecXML fail, no/bad response");
    }

    return scResponse;
}

void FlightController::ConnectMavLink()
{
    socketDesc = socket(AF_INET, SOCK_STREAM, 0);
    if (socketDesc == -1) {
        printL("FlightController::ConnectMavLink fail socket");
    }

    addressMavLink.sin_addr.s_addr = inet_addr(scMavLinkAddress);
    addressMavLink.sin_family = AF_INET;
    addressMavLink.sin_port = htons(iMavlinkPort);

    //Connect to remote server
    if (connect(socketDesc, (struct sockaddr *)&addressMavLink, sizeof(addressMavLink)) < 0) {
        printL("FlightController::ConnectMavLink fail connect");
    }
}

/*

Drive_TwinTrackAdv::Drive_TwinTrackAdv(std::vector<Navigator*>& arNav)
: DrivePlanner(arNav)
, pCurrentRoute(nullptr)
, fDistBeforeChangeDest(1000)
, iDestAvgCursor(0)
, iSpeedAvgCursor(0)
, iOrientationAvgCursor(0)
{
    CameraController::cameraFeatures features = CameraController::cameraFeatures(CameraController::CameraController_RGB | CameraController::CameraController_IR | CameraController::CameraController_Align);
    pCamera = CameraController::OpenCamera(CameraController::CameraController_RS, features);
    if (pCamera == nullptr) {
        printf("Error opening camera\n");
        return;
    }

    try {     
        pthrCamera = new std::thread(&CameraController::run, pCamera, (void*)&twinTrackAdv_CameraCallback, (void*)this);
    } catch (rs2::error e) {
        std::cout << "Camera Error. Type: " << e.get_type() << std::endl;
        std::cout << "Camera Error. Function: " << e.get_failed_function() << std::endl;
        std::cout << "Camera Error. Args: " << e.get_failed_args() << std::endl;

        static int iRetryCount = 0;
        // Try 3 times to restart the camera
        if (iRetryCount++ < 3) {
            std::cout << "Camera Error. Restart attempt: " << iRetryCount << std::endl;
            //figure out if thread is running...if so stop it
            delete pCamera;
            pCamera = CameraController::OpenCamera(CameraController::CameraController_RS, features);
            if (pCamera == nullptr) {
                std::cout << "Camera Error. Unable to create new camera.\n";
            }
        }
    }
}
*/
