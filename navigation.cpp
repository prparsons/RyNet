#include "navigation.h"

#include <thread>
#include <unistd.h>


// ** ObjectDetection

ObjectDetection::ObjectDetection(DataLogger& _logger)
: logger(_logger)
{
	// For Jetson Nano (just let it use gpu)
    pNetwork = nullptr;//detectNet::Create(detectNet::SSD_MOBILENET_V2);
    // SSD_INCEPTION_V2 ?
	// For Xavier (try to use one of the DLA engines so we can use the gpu for opencv stuff)
	//detectNet* net = detectNet::Create(detectNet::SSD_MOBILENET_V2, 0.5f, 1, TYPE_FASTEST, DEVICE_DLA_0);DEVICE_DLA_1 DEVICE_GPU
}

void ObjectDetection::SetNewImage(image& img)
{
    mt.lock();
    imgRecent = img;
    mt.unlock();
    sigNewImage.signal();
}

bool ObjectDetection::ProcessImage(image& imgRGB)
{
    printf("ObjectDetection::ProcessImage");
    // these variables will store the image data pointer and dimensions
    uchar3* u3Img = (uchar3*)imgRGB.data;   // shared CPU/GPU pointer to image
    int imgWidth = imgRGB.cols;      // width of the image (in pixels)
    int imgHeight = imgRGB.rows;      // height of the image (in pixels)

	// For Jetson Nano (just let it use gpu)

	detectNet::Detection* arDetections = nullptr;
	int iDetections = pNetwork->Detect(u3Img, imgWidth, imgHeight, &arDetections);

    if (iDetections < 1) {
        //printf("no things");
        return false;
    }

    bool bRet = false;
    char sc[128];
    for (int i = 0; i < iDetections; i++) {
        //if (arDetections[i].Confidence < 0.5) continue; // This isn't necessary since the argument is passed to detectNet
        if (arDetections[i].ClassID != 1) continue; // Should be a person (looking for people)

        bRet = true;
        snprintf(sc, 127, "Detection Class[%d] LTRB:%d %d %d %d", 
                int(arDetections[i].ClassID), 
                int(arDetections[i].Left), 
                int(arDetections[i].Top), 
                int(arDetections[i].Right), 
                int(arDetections[i].Bottom));
        logger.LogString(sc);
    }

	// Not sure if this is right for freeing the memory
	cudaFree(arDetections);
    return bRet;
}

void ObjectDetection::run()
{
    printf("ObjectDetection::run");
    Signal* pSigRGB = new Signal(sigNewImage);
    image imgProcess;

    while (!ss.shouldStop()) {
        pSigRGB->wait();
        mt.lock();
        imgProcess = imgRecent;
        mt.unlock();
        
        //if (ProcessImage(imgProcess)) {
        //    logger.LogImage(imgProcess);
       // }
    }
    delete pSigRGB;
}

// ** TerrainSegmentation

TerrainSegmentation::TerrainSegmentation(DataLogger& _logger)
: logger(_logger)
{
    pNetwork = nullptr;//segNet::Create(segNet::FCN_RESNET18_SUNRGB_640x512);//segNet::FCN_RESNET18_DEEPSCENE_576x320);FCN_ALEXNET_AERIAL_FPV_720p
}

bool TerrainSegmentation::ProcessImage(image& imgRGB)
{
    printf("TerrainSegmentation::ProcessImage");
    uchar3* u3Img = (uchar3*)imgRGB.data;   // shared CPU/GPU pointer to image
    int imgWidth = imgRGB.cols;      // width of the image (in pixels)
    int imgHeight = imgRGB.rows;      // height of the image (in pixels)

    if(!pNetwork->Process(u3Img, imgWidth, imgHeight)) {
        printf("TerrainSegmentation::ProcessImage fail");
        return false;
    }
    
    if(!pNetwork->Mask(u3Mask, imgWidth, imgHeight, segNet::FILTER_POINT)) {
        return false;
    }

    return ScanMask(u3Mask, imgWidth, imgHeight);
}

float TerrainSegmentation::GetHabitatDirection()
{
    return 1.0;
}

void TerrainSegmentation::SetNewImage(image& img)
{
    mt.lock();
    imgRecent = img;
    mt.unlock();
    sigNewImage.signal();
}

void TerrainSegmentation::run()
{
    printf("TerrainSegmentation::run");
    Signal* pSigRGB = new Signal(sigNewImage);
    image imgProcess;

    while (!ss.shouldStop()) {
        pSigRGB->wait();
        mt.lock();
        imgProcess = imgRecent;
        mt.unlock();
        
        //if (ProcessImage(imgProcess)) {
        //    logger.LogImage(imgProcess);
        //    logger.LogString("Good Habitat Found");
        //}
    }
    delete pSigRGB;
}

bool TerrainSegmentation::ScanMask(uchar3* imgMask, int iWidth, int iHeight)
{
    printf("TerrainSegmentation::ScanMask");
    int size = iWidth * iHeight;
    float fAngleAvgSum = 0;
    int iAngleAvgCount = 0;
    uint32_t pixel;
    for (int i = 0; i < size; i++) {
        pixel = *((uint32_t*)(&imgMask[i])) >> 8;
        if (pixel == HABITAT) {
            int x = i / iWidth;
            fAngleAvgSum += (abs(x - (iWidth * .5)) / (iWidth * .5)) * (fImageFOV * .5);
            iAngleAvgCount++;
        }
    }

    if (iAngleAvgCount == 0) {
        fBestDirection = -1;
        return false;
    }

    // We've found the desired category
    logger.LogMask(imgMask, iWidth, iHeight);
    fBestDirection = fAngleAvgSum / iAngleAvgCount;
    return true;
}

Navigator::Navigator(FlightController& _control, DataLogger& _logger, CameraController& _camera)
: control(_control)
, logger(_logger)
, camera(_camera)
, pThreadNav(nullptr)
, pThreadDet(nullptr)
, pThreadSeg(nullptr)
, detector(logger)
, segmenter(logger)
{

}

void Navigator::runNavigator()
{
    printf("Navigator::runNavigator");
    cameraFrame* pFrame = nullptr;
    Signal* pSigRGB = camera.GetSignalRGB();

    bool bGridSearching = false;
    float fMargin = 0;
    float fCurrentDirection = 0;

    std::thread thrObject(&ObjectDetection::run, &detector);
    std::thread thrSegment(&TerrainSegmentation::run, &segmenter);

    while (!ssMain.shouldStop()) {
        pSigRGB->wait();
        if (pFrame) delete pFrame;
        pFrame = camera.GetCurrentFrame();
        if (!pFrame->hasRGB()) continue;
        
        image imgNew = pFrame->getImageRGB();
        detector.SetNewImage(imgNew);
        segmenter.SetNewImage(imgNew);

        if (bGridSearching) {
            if (segmenter.GetHabitatDirection() >= 0.0) { // good terrain visible?
                control.SetDirection(segmenter.GetHabitatDirection());
                fCurrentDirection = segmenter.GetHabitatDirection();
                char sc[64];
                sprintf(sc, "Grid Search -> Good Habitat %1.3f", fCurrentDirection);
                logger.LogString(sc);
            }
        } else {
            float fDiff = abs(segmenter.GetHabitatDirection() - fCurrentDirection);
            if (fDiff < fMargin) continue; // good terrain in current direction?
            if (segmenter.GetHabitatDirection() >= 0.0) { // good terrain in different direction?
                control.SetDirection(segmenter.GetHabitatDirection());
                fCurrentDirection = segmenter.GetHabitatDirection();
                char sc[64];
                sprintf(sc, "Grid Search Change -> %1.3f", fCurrentDirection);
                logger.LogString(sc);
            } else {
                control.RestoreRouteNaive();
                logger.LogString("Good Habitat -> Grid Search");
            }
        }
    }
    if (pFrame != nullptr) delete pFrame;
    delete pSigRGB;

    thrObject.join();
    thrSegment.join();
}

#ifdef dontdefine
class Navigator_PersonFinder {
public:
    Navigator_PersonFinder();
    virtual ~Navigator_PersonFinder();
    
    virtual void newImageRGB(image imgRGB, bool bSignal) override;
    virtual void newImageDepthRaw(cameraFrame_depth depthRaw, bool bSignal) override;

    virtual void _run(Signal* psigMain) override;

protected:
    virtual float _getConfidenceDest() override;

private:
    bool process();

    bool bNewImgRGB;
    bool bNewImgDepthRaw;
    Signal* psigImages;
    image imgRGB;
    cameraFrame_depth *pDepthRaw;
    ThreadMutex mtImages;
    detectNet* pNetwork;
};


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
    fXDestination *= (PI / 180); // Convert to radians
    // Now we have angle and distance, need the 2 legs of the triangle now
    float fX = fDistanceMeters * sin(fXDestination);
    float fY = fDistanceMeters * cos(fXDestination);
    dest.fX = fX;
    dest.fY = fY;
    
	// Not sure if this is right for freeing the memory
	cudaFree(arDetections);

	return true;
}

#endif
