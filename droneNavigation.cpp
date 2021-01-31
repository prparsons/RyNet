#include "droneNavigation.h"

#include <stdio.h>
#include <thread>
#include <unistd.h>
#include "cameraMulti.h"


#define MIN_RANGE 0.6
//#define FRAME_WIDTH 848
//#define FRAME_HEIGHT 480

void updateDepthCamera(cameraFrame* pFrame, void* pNavigator)
{
    if (pFrame == nullptr || pNavigator == nullptr) {
        printf("@updateDepthCamera:arguments were null\n");
        return;
    }
    Navigator& nav = *((Navigator*)pNavigator);
    // hasRGB, hasIR, hasDepth, hasREdge; getImageRGB
    if (!pFrame->hasRGB()) {
        printf("updateDepthCamera:frame has no rgb\n");
        return;
    }
    bool bDepth = pFrame->hasDepth();
    bool bIR = pFrame->hasIR();
    bool bREdge = pFrame->hasREdge();
    image imageRGB = pFrame->getImageRGB();
    nav.newImageRGB(imageRGB, !(bDepth || bIR || bREdge));
    if (bDepth) {
        image imageDepth = pFrame->getImageDepth();
        nav.newImageDepth(imageDepth, false);
        nav.newImageDepthRaw(pFrame->getImageDepthRaw(), !(bIR || bREdge));
    }
    if (bIR) {
        image imageIR = pFrame->getImageIR();
        nav.newImageIR(imageIR, !bREdge);
    } 
    if (bREdge) {
        image imageREdge = pFrame->getImageREdge();
        nav.newImageREdge(imageREdge, true);
    }
}

float activ_null(const float& f) { return f; }


// *********
// Navigator

Navigator::Navigator(float (*_activ)(const float&))
: activ(_activ)
{

}

// *****
// Drive

Drive::Drive(std::vector<Navigator*>& _arNav)
: arNav(_arNav)
, fSpeed(0)
, fSteer(0)
, fClimb(0)
{
}

// **************************
// Navigator_SimpleObstruct2D

Navigator_SimpleObstruct2D::Navigator_SimpleObstruct2D()
: Navigator(&activ_null)
, bNewImgRGB(false)
, bNewImgDepth(false)
, bNewImgDepthRaw(false)
, pCamera(nullptr)
, pDepthRaw(nullptr)
{
    fSpeed = 0.5;

    cameraMulti::cameraFeatures features = cameraMulti::cameraFeatures(cameraMulti::cameraMulti_RGB | cameraMulti::cameraMulti_Depth);
    pCamera = cameraMulti::OpenCamera(cameraMulti::cameraMulti_RS, features);
    if (pCamera == nullptr) {
        printf("Error opening camera\n");
        return;
    }
}

Navigator_SimpleObstruct2D::~Navigator_SimpleObstruct2D()
{
    if (pCamera != nullptr) {
        delete pCamera;
    }
    if (pDepthRaw) {
        delete pDepthRaw;
    }
}

void Navigator_SimpleObstruct2D::newImageRGB(image _imgRGB, bool bSignal)
{
    ThreadLock locker(mtImages);
    locker.lock();
    bNewImgRGB = true;
    imgRGB = _imgRGB; // copying is slow...need a single shareable read instance
    locker.unlock();
    if (bSignal) {
        if (psigImages != nullptr) {
            psigImages->signal();
        }
    }
}

void Navigator_SimpleObstruct2D::newImageDepth(image _imgDepth, bool bSignal)
{
    ThreadLock locker(mtImages);
    locker.lock();
    bNewImgDepth = true;
    imgDepth = _imgDepth;
    locker.unlock();
    if (bSignal) {
        if (psigImages != nullptr) {
            psigImages->signal();
        }
    }
}

void Navigator_SimpleObstruct2D::newImageDepthRaw(cameraFrame_depth _depthRaw, bool bSignal)
{ 
    ThreadLock locker(mtImages);
    locker.lock();
    bNewImgDepthRaw = true;
    if (pDepthRaw) delete pDepthRaw;
    pDepthRaw = new cameraFrame_depth(_depthRaw);
    locker.unlock();
    if (bSignal) {
        if (psigImages != nullptr) {
            psigImages->signal();
        }
    }
}

bool Navigator_SimpleObstruct2D::processTerrain()
{
    return false; // This one isn't bothering with terrain
}

bool Navigator_SimpleObstruct2D::processObstruct()
{
    ThreadLock locker(mtImages);
    locker.lock();
    if (pDepthRaw == nullptr) {
        printf("Not any raw data\n");
        return false;
    }
    cameraFrame_depth depthRaw(*pDepthRaw);
    locker.unlock();
    {
        //scoped_timer Timer("detect OpenMP loop");
    bool arRegions[4][4];
        // Use decimation filter to downsample
        //rs2::decimation_filter downsample(8);
        //frame = frame.apply_filter(downsample).as<rs2::depth_frame>();
    const int iRegionWidth = depthRaw.getWidth() / 4;
    const int iRegionHeight = depthRaw.getHeight() / 4;
        // Dividing the frame in 16 sections (4x4)
        //const int iRegionWidth = frame.get_width() / 4;
        //const int iRegionHeight = frame.get_height() / 4;
        // Return true if we find anything
        bool bReturn = false;
        // For each region find the minimum value in that region. Ignore zeroes,
        //  since that's what RS uses for invalid values (need to verify this)
        
        // Can we use any simd here?
        // With the decimation filter enabled threading slows things down
        //#pragma omp parallel for
        for (int iRegionX = 0; iRegionX < 4; iRegionX++) {
            for (int iRegionY = 0; iRegionY < 4; iRegionY++) {
                const int iStartX = iRegionX * iRegionWidth;
                //const int iEndX = iRegionX == 3 ? frame.get_width() : iStartX + iRegionWidth;
    const int iEndX = iRegionX == 3 ? depthRaw.getWidth() : iStartX + iRegionWidth;
                bool bClose = false;
    point pt;
    pt.iXY[0] = iStartX;
    for (int& x = pt.iXY[0]; x < iEndX; x++) {
                //for (int x = iStartX; x < iEndX; x++) {
                    const int iStartY = iRegionY * iRegionHeight;
                    //const int iEndY = iRegionY == 3 ? frame.get_height() : iStartY + iRegionHeight;
    const int iEndY = iRegionY == 3 ? depthRaw.getHeight() : iStartY + iRegionHeight;
    pt.iXY[1] = iStartY;
    for (int& y = pt.iXY[1]; y < iEndY; y++) {
                    //for (int y = iStartY; y < iEndY; y++) {
                        //float fValue = frame.get_distance(x, y);
    float fValue = depthRaw[{x, y}];
                        if ((fValue < MIN_RANGE) && (fValue != 0.0)) {
                            bClose = true;
                            break;
                        }
                    }
                    if (bClose) {
                        break;
                    }
                }
                arRegions[iRegionX][iRegionY] = bClose;
                bReturn = bReturn || bClose;
            }
        }

    if (bReturn) {
        for (int i = 1; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (arRegions[i][j]) {
                    fSteer = -1.0;
                    return true;
                }
            }
        }
        fSteer = 0;
        return false;
    }
    }
    fSteer = 0;
    return false; // This should figure out speed, and turn left if something is in the way
}

bool Navigator_SimpleObstruct2D::processAnimals()
{
    return false; // Use this possibly for altitude?
}

void Navigator_SimpleObstruct2D::_run(Signal* psigMain)
{
    try {
        Signal& sigMain = *psigMain;
        Signal sigImages;
        psigImages = new Signal(sigImages);

        if (pCamera == nullptr) {
            printf("No camera\n");
            return;
        }
        //std::thread thrCamera(*pCamera, (void*)&updateDepthCamera, (void*)this);
        std::thread thrCamera(&cameraMulti::run, pCamera, (void*)&updateDepthCamera, (void*)this);

        do {
            sigImages.wait();
            
            ThreadLock locker(mtImages);
            locker.lock();
            if (!(bNewImgDepth && bNewImgDepthRaw && bNewImgRGB)) {
                locker.unlock();
                continue; // Wait until new images of both
            }
            bNewImgDepth = false;
            bNewImgRGB = false;
            bNewImgDepthRaw = false;
            locker.unlock();
            
            // Very first thing check for obstructions. If there are, immediately signal that speed/steer have been affected
            if (processObstruct()) {
                flagDirty |= Navigator::SPEED | Navigator::STEER;
                sigMain.signal();
            }
            // After that, check terrain
            if (processTerrain()) {
                // If somewhere else looks better, flag steering
                flagDirty |= Navigator::STEER;
                sigMain.signal();
            }
            // After navigation, see if we detected any animals (caching training images)
            processAnimals();
        } while(!ss.shouldStop());
        pCamera->ss.stop();
        thrCamera.join();
        delete psigMain;
        delete psigImages;
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
            ss.reset();
            pCamera = cameraMulti::OpenCamera(cameraMulti::cameraMulti_RS, cameraMulti::cameraFeatures(cameraMulti::cameraMulti_RGB | cameraMulti::cameraMulti_Depth));
            if (pCamera != nullptr) {
                delete psigMain;
                //this needs to be a function I can call
            } else {
                std::cout << "Camera Error. Unable to create new camera.\n";
            }
        }
    }
}

float Navigator_SimpleObstruct2D::_getConfidenceRaw()
{
    return 1.0;
}

// ***************
// Drive_TwinTrack

Drive_TwinTrack::Drive_TwinTrack(std::vector<Navigator*>& arNav)
: Drive(arNav)
{
}

bool Drive_TwinTrack::_decide(Navigator&, Navigator::dirty)
{
    return _decide();
}

bool Drive_TwinTrack::_decide()
{
    // Tell the nav it isn't dirty anymore
    float _fSteer = arNav[0]->getSteer();
    float _fSpeed = arNav[0]->getSpeed();

    bool bReturn = (fSpeed != _fSpeed) || (fSteer != _fSteer);

    fSpeed = _fSpeed;
    fSteer = _fSteer;
    return bReturn;
}

bool sendXML(const char* strMessage)
{
	int socket_desc;
	struct sockaddr_in server;
	
	//Create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	if (socket_desc == -1)
	{
		printf("Could not create socket");
	}
		
	server.sin_addr.s_addr = inet_addr(HOST);
	server.sin_family = AF_INET;
	server.sin_port = htons( PORT );

	//Connect to remote server
	if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
	{
		puts("connect error");
		return 1;
	}
	
	puts("Connected\n");
	
	//Send some data
	if( send(socket_desc , strMessage , strlen(strMessage) , 0) < 0)
	{
		puts("Send failed");
		return false;
	}
	puts("Data Send\n");
	
	return true;
}