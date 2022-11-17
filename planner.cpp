#include "planner.h"

#include <vector>

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

bool Drive_TwinTrackAdv::processDestRunningAvg(Destination dest)
{
    arDestAvg[iDestAvgCursor] = dest;
    float fX = 0;
    float fY = 0;
    float fZ = 0;
    //SIMD candidate
    for (int i = 0; i < arDestAvg.size(); i++) {
        fX += arDestAvg[i].fX;
        fY += arDestAvg[i].fY;
        fZ += arDestAvg[i].fZ;
    }
    fX /= arDestAvg.size();
    fY /= arDestAvg.size();
    fZ /= arDestAvg.size();
    if (++iDestAvgCursor == arDestAvg.size()) iDestAvgCursor = 0;

    Destination destCurrent = arRouteList[arRouteList.size() - 1]->GetRoute().GetLocation(1.0);
    float fDistance = sqrt(((fX - destCurrent.fX) * (fX - destCurrent.fX)) +
                           ((fY - destCurrent.fY) * (fY - destCurrent.fY)) +
                           ((fZ - destCurrent.fZ) * (fZ - destCurrent.fZ)));
    if (fDistance > fDistBeforeChangeDest) {
        destAvg.fX = fX;
        destAvg.fY = fY;
        destAvg.fZ = fZ;
        return true;
    }
    return false;
}
*/
