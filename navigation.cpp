#include "navigation.h"

#include <jetson-inference/detectNet.h>
#include <jetson-utils/loadImage.h>
#include <thread>
#include <unistd.h>

#include "data_types.h"
#include "serverDroneStatus.h"

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
    flagDirty |= Navigator::SPEED | Navigator::STEER;

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
    static float fLastSteer = 0;
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
                    orient.fYaw = -1.0;
                    if (fLastSteer == orient.fYaw) return false;
                    fLastSteer = orient.fYaw;
                    return true;
                }
            }
        }
        orient.fYaw = 0;
        if (fLastSteer == orient.fYaw) return false;
        fLastSteer = orient.fYaw;
        return true;
    }
    }
    orient.fYaw = 0;
    if (fLastSteer == orient.fYaw) return false;
    fLastSteer = orient.fYaw;
    return true;// This should figure out speed, and turn left if something is in the way
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
        
        std::thread thrCamera(&cameraMulti::run, pCamera, (void*)&updateDepthCamera, (void*)this);

        do {
            sigImages.wait(1000);
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
                printf("Nav loop dirty obstruction\n");
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

Navigator_PersonFinder::Navigator_PersonFinder()
: Navigator(&activ_null)
, bNewImgRGB(false)
, psigImages(nullptr)
, pDepthRaw(nullptr)
, pNetwork(nullptr)
{

}

Navigator_PersonFinder::~Navigator_PersonFinder()
{
    if (pNetwork != nullptr) delete pNetwork;
    if (psigImages != nullptr) delete psigImages;
    if (pDepthRaw != nullptr) delete pDepthRaw; // do we need to lock this (what thread is the update called from...need to make sure camer is stopped?)
}
    
void Navigator_PersonFinder::newImageRGB(image _imgRGB, bool bSignal)
{
    ThreadLock locker(mtImages);
    locker.lock();
    if (bNewImgRGB) {
        // cout << "New image when previous not used"; // Don't want to use unsynchronized frames, so we'll wait until the depth has finished
        locker.unlock();
        return;
    }
    bNewImgRGB = true;
    imgRGB = _imgRGB; // copying is slow...need a single shareable read instance
    locker.unlock();
    if (bSignal) {
        if (psigImages != nullptr) {
            psigImages->signal();
        }
    }
}

void Navigator_PersonFinder::newImageDepthRaw(cameraFrame_depth _depthRaw, bool bSignal)
{ 
    ThreadLock locker(mtImages);
    locker.lock();
    if (bNewImgDepthRaw) {
        // cout << "New image when previous not used"; // Don't want to use unsynchronized frames, so we'll wait until the rgb has finished
        locker.unlock();
        return;
    }
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

void Navigator_PersonFinder::_run(Signal* psigMain)
{
    Signal& sigMain = *psigMain;
    Signal sigImages;
    psigImages = new Signal(sigImages);

    do {
        sigImages.wait(1000);
        ThreadLock locker(mtImages);
        locker.lock();
        if (!(bNewImgRGB && bNewImgDepthRaw)) {
            locker.unlock();
            continue; // Wait until new images of both
        }
           
        if (process()) {
            flagDirty |= Navigator::DEST_X;
            flagDirty |= Navigator::DEST_Y;
            if (fSpeed == 0) {
                fSpeed = 1.0;
                flagDirty |= Navigator::SPEED;
            }
            sigMain.signal();
        } else if (fSpeed != 0) {
            fSpeed = 0;
            flagDirty |= Navigator::SPEED;
            sigMain.signal();
        }
        // Unlock and reset everything now that we're done with images
        bNewImgRGB = false;
        bNewImgDepthRaw = false;
        locker.unlock();

    } while(!ss.shouldStop());
    delete psigMain;
    delete psigImages;
}

float Navigator_PersonFinder::_getConfidenceDest()
{
    return 1.0f;
}

bool Navigator_PersonFinder::process()
{
    // these variables will store the image data pointer and dimensions
    uchar3* imgPtr = (uchar3*)imgRGB.data;   // shared CPU/GPU pointer to image
    int imgWidth = imgRGB.cols;      // width of the image (in pixels)
    int imgHeight = imgRGB.rows;      // height of the image (in pixels)

	// For Jetson Nano (just let it use gpu)
    
    if (!pNetwork) pNetwork = detectNet::Create(detectNet::SSD_MOBILENET_V2);
    // SSD_INCEPTION_V2 ?
	// For Xavier (try to use one of the DLA engines so we can use the gpu for opencv stuff)
	//detectNet* net = detectNet::Create(detectNet::SSD_MOBILENET_V2, 0.5f, 1, TYPE_FASTEST, DEVICE_DLA_0);

	// check to make sure that the network model loaded properly
	if (!pNetwork) {
		printf("failed to load image recognition network\n");
		return 0;
	}

	detectNet::Detection* arDetections = nullptr;
	int iDetections = pNetwork->Detect(imgPtr, imgWidth, imgHeight, &arDetections);

    if (iDetections < 1) {
        //printf("No People");
        return false;
    }

    float fConvertX = pDepthRaw->getWidth() / float(imgWidth);
    float fConvertY = pDepthRaw->getHeight() / float(imgHeight);
    float fMinDistance = MAXFLOAT;
    int iXDestination = 0;
    for (int i = 0; i < iDetections; i++) {
        //if (arDetections[i].Confidence < 0.5) continue; // This isn't necessary since the argument is passed to detectNet
        if (arDetections[i].ClassID != 1) continue; // Should be a person (looking for people)

        DetectionBox bounds;
        bounds.fLeft = arDetections[i].Left / float(pDepthRaw->getWidth());
        ServerLoop::GetServerLoopSingleton().AddDetection(bounds);

        float fTop = arDetections[i].Top * fConvertY;
        float fRight = arDetections[i].Right * fConvertX;
        float fBottom = arDetections[i].Bottom * fConvertY;
        float fLeft = arDetections[i].Left * fConvertX;

        // Should we bother rounding?
        for (int x = fLeft; x < fRight; x++) {
            for (int y = fTop; y < fBottom; y++) {
                point pt;
                pt.iXY[0] = x;
                pt.iXY[1] = y;
                if ((*pDepthRaw)[pt] < fMinDistance) {
                    fMinDistance = (*pDepthRaw)[pt];
                    iXDestination = x;
                }
            }
        }
    }
    if (fMinDistance == MAXFLOAT) {
        //printf("not high enough confidence");
        return false;
    }

    float fDistanceMeters = fMinDistance * pDepthRaw->getMetersPerUnit();
    float fXDestination = iXDestination / fConvertX;
    fXDestination /= imgWidth; //percentage
    fXDestination -= 0.5; //0 at center of image
    fXDestination *= (69 / 2); // FOV horizontal / 2
    //CONVERT DEGREES TO RADIANS!!!!
    // Now we have angle and distance, need the 2 legs of the triangle now
    float fX = fDistanceMeters * sin(fXDestination);
    float fY = fDistanceMeters * cos(fXDestination);
    dest.fX = fX;
    dest.fY = fY;
    
	// Not sure if this is right for freeing the memory
	cudaFree(arDetections);

	return true;
}

Navigator_SimpleDepthObstruct::Navigator_SimpleDepthObstruct()
: Navigator(&activ_null)
, bNewImgDepthRaw(false)
, pDepthRaw(nullptr)
{
    
}

Navigator_SimpleDepthObstruct::~Navigator_SimpleDepthObstruct()
{
    if (psigImages != nullptr) delete psigImages;
    if (pDepthRaw != nullptr) delete pDepthRaw; // do we need to lock this (what thread is the update called from...need to make sure camer is stopped?)
}

void Navigator_SimpleDepthObstruct::newImageDepthRaw(cameraFrame_depth _depthRaw, bool bSignal)
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

bool Navigator_SimpleDepthObstruct::getObstructions(std::vector<Obstruction>& _arObs)
{
    if (arObs.size() == 0) return false;
    _arObs = arObs;
    return true;
}

void Navigator_SimpleDepthObstruct::_run(Signal* psigMain)
{
    Signal& sigMain = *psigMain;
    Signal sigImages;
    psigImages = new Signal(sigImages);

    do {
        sigImages.wait(1000);
        ThreadLock locker(mtImages);
        locker.lock();
        if (!bNewImgDepthRaw) {
            locker.unlock();
            continue; // Wait until new images of both
        }
           
        if (process()) {
            flagDirty |= Navigator::SPEED;
            sigMain.signal();
        }
        
        // Unlock and reset everything now that we're done with images
        bNewImgDepthRaw = false;
        locker.unlock();

    } while(!ss.shouldStop());
    delete psigMain;
    delete psigImages;
}

bool Navigator_SimpleDepthObstruct::process()
{
    ThreadLock locker(mtImages);
    locker.lock();
    if (pDepthRaw == nullptr) {
        printf("Not any raw data\n");
        return false;
    }
    cameraFrame_depth depthRaw(*pDepthRaw);
    locker.unlock();
    
    arObs.clear();
    float arRegions[4][4];
    // Use decimation filter to downsample
    depthRaw.applyDecimationFilter();
    // Dividing the frame in 16 sections (4x4)
    const int iRegionWidth = depthRaw.getWidth() / 4;
    const int iRegionHeight = depthRaw.getHeight() / 4;

    // For each region find the minimum value in that region. Ignore zeroes,
    //  since that's what RS uses for invalid values
        
    // With the decimation filter enabled threading slows things down
    //#pragma omp parallel for
    for (int iRegionX = 0; iRegionX < 4; iRegionX++) {
        for (int iRegionY = 0; iRegionY < 4; iRegionY++) {
            const int iStartX = iRegionX * iRegionWidth;
            const int iEndX = iRegionX == 3 ? depthRaw.getWidth() : iStartX + iRegionWidth;
            float fMin = MAXFLOAT;
            point pt;
            pt.iXY[0] = iStartX;
            for (int& x = pt.iXY[0]; x < iEndX; x++) {
                const int iStartY = iRegionY * iRegionHeight;
                const int iEndY = iRegionY == 3 ? depthRaw.getHeight() : iStartY + iRegionHeight;
                pt.iXY[1] = iStartY;
                for (int& y = pt.iXY[1]; y < iEndY; y++) {
                    float fValue = depthRaw[pt];
                    if (fValue < fMin) {
                        fMin = fValue;
                    }
                }
            }
            arRegions[iRegionX][iRegionY] = fMin;
        }
    }

    bool bReturn = false;
    // Looking at the full width, ignoring bottom row (floor/ground)
    //  0,0     1,0     2,0     3,0
    //  0,1     1,1     2,1     3,1
    //  0,2     1,2     2,2     3,2
    //  x       x       x       x
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            if (arRegions[i][j] < 10) { // Only consider things < 10 meters
                //create obstruction
                //Obstruction(DetectionBox box, float fTotalWidth, float fHorizFOV)
                DetectionBox box;
                box.fLeft = i * iRegionWidth;
                box.fRight = box.fLeft + iRegionWidth;
                box.fDistance = arRegions[i][j];
                arObs.push_back(Obstruction(box, depthRaw.getWidth(), 87));
                bReturn = true;
            }
        }
    }

    return bReturn;
}
