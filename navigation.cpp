#include "navigation.h"

#include <thread>
#include <unistd.h>
#include <jetson-utils/cudaMappedMemory.h>

detectNet::NetworkType networkDetection = detectNet::SSD_MOBILENET_V2;
segNet::NetworkType networkSegmentation = segNet::FCN_RESNET18_VOC_512x320;

// ** ObjectDetection

ObjectDetection::ObjectDetection(DataLogger& _logger)
: iNumSlices(3)
, logger(_logger)
, pNetwork(nullptr)
, pThread(nullptr)
{
    pNetwork = detectNet::Create(networkDetection);
	//detectNet* net = detectNet::Create(detectNet::SSD_MOBILENET_V2, 0.5f, 1, TYPE_FASTEST, DEVICE_DLA_0);DEVICE_DLA_1 DEVICE_GPU
    // DLA fails with segmentation fault using detectnet with that network. even resizing images
}

ObjectDetection::~ObjectDetection()
{
    if (pNetwork != nullptr) delete pNetwork;
    if (pThread != nullptr) {
        ssMain.stop();
        if (pThread->joinable()) pThread->join();
        delete pThread;
    }
}

void ObjectDetection::SetNewImage(CameraFrame& frm)
{
    mt.lock();
    frmRecent.DeepCopy(frm);
    mt.unlock();
    sigNewImage.signal();
}

bool ObjectDetection::ProcessImage(CameraFrame& frm)
{
    cv::Mat& _imgRGB = frm.matrix;
    cv::Mat imgRGB;
    cv::resize(_imgRGB, imgRGB, cv::Size(_imgRGB.cols / 2, _imgRGB.rows / 2), cv::INTER_LINEAR);
    // 1080x1920 -> 540x960  --- 3 images at 540x320

    int imgWidth = imgRGB.cols;      // width of the image (in pixels)
    int imgHeight = imgRGB.rows;      // height of the image (in pixels)
    // Going to do iNumSlices horizontal slices of the image. 
    if (imgHeight % iNumSlices != 0) printL("ObjectDetection::ProcessImage doesn't divide by 3");
    int iSliceHeight = imgHeight / iNumSlices;
    uchar3* u3Img;
    if (CUDA_FAILED(cudaMalloc(&u3Img, imgWidth * iSliceHeight * 3))) {
        printL("ObjectDetection::ProcessImage cudaMalloc fail");
        return false;
    }
    bool bRet = false;
    for (int s = 0; s < iNumSlices; s++) {
        intptr_t imgRGBOffset = s * imgWidth * imgHeight;
        cudaMemcpy(u3Img, ((uint8_t*)imgRGB.data) + imgRGBOffset, imgWidth * iSliceHeight * 3, cudaMemcpyHostToDevice);   // shared CPU/GPU pointer to image
        detectNet::Detection* arDetections = nullptr;
        int iDetections = pNetwork->Detect(u3Img, imgWidth, iSliceHeight, &arDetections);
        if (iDetections < 0) {
            // Error processing image
            cudaFree(u3Img);
            return false;
        } else if (iDetections == 0) {
            continue;
        }

        char sc[128];
        for (int i = 0; i < iDetections; i++) {
            bRet = true;
            snprintf(sc, 127, "ImageId, Detection Slice,Class,Confidence[%d,%d,%d,%d] LTRB:%d %d %d %d", 
                    frm.GetId(),
                    s,
                    int(arDetections[i].ClassID), 
                    int(arDetections[i].Confidence * 100),
                    int(arDetections[i].Left), 
                    int(arDetections[i].Top), 
                    int(arDetections[i].Right), 
                    int(arDetections[i].Bottom));
            logger.LogString(sc);
        }
    }

    cudaFree(u3Img);
    return bRet;
}

void ObjectDetection::run(Signal sig, StopSignal ss)
{
    printL("ObjectDetection::run");
    CameraFrame frmProcess;
    int iFramesProcessed = 0;
    while (!ss.shouldStop()) {
        sig.wait(1000);
        mt.lock();
        frmProcess = frmRecent; // This doesn't need to be a deep copy
        mt.unlock();
        if (frmProcess.matrix.empty()) continue;
        iFramesProcessed++;

        if (ProcessImage(frmProcess)) {
            logger.LogImage(frmProcess, "detect");
        }
    }
    printf("ObjectDetection::run Frames Processed:%d\n", iFramesProcessed);
}

void ObjectDetection::Start()
{
    pThread = new std::thread(&ObjectDetection::run, this, sigNewImage, ssMain);
}

void ObjectDetection::SendStopSignal()
{
    ssMain.stop();
}

void ObjectDetection::SetNumberImageSlices(int _iNumSlices)
{
    iNumSlices = _iNumSlices;
}

bool ObjectDetection::SetNetwork(const char* scNetwork)
{
    if (strcmp(scNetwork, "SSD_MOBILENET_V2") == 0) {
        networkDetection = detectNet::SSD_MOBILENET_V2;
    } else if (strcmp(scNetwork, "SSD_MOBILENET_V1") == 0) {
        networkDetection = detectNet::SSD_MOBILENET_V1;
    } else if (strcmp(scNetwork, "SSD_INCEPTION_V2") == 0) {
        networkDetection = detectNet::SSD_INCEPTION_V2;
    } else {
        return false;
    }
    return true;
}

// ** TerrainSegmentation

TerrainSegmentation::TerrainSegmentation(DataLogger& _logger)
: bHabitatVisible(false)
, uTargetClass(HABITAT)
, iNumSlices(3)
, fImageFOV(50) // I calculated this number based on the imx477 sensor size with a 6mm lens
, logger(_logger)
, pMask(nullptr)
, pThread(nullptr)
, uMaskWidth(0)
, uMaskHeight(0)
, uValidNumPixels(18)
{
    pNetwork = segNet::Create(networkSegmentation);//FCN_RESNET18_SUNRGB_640x512);//segNet::FCN_RESNET18_DEEPSCENE_576x320);FCN_ALEXNET_AERIAL_FPV_720p
//  static segNet* Create( NetworkType networkType=FCN_ALEXNET_CITYSCAPES_SD, uint32_t maxBatchSize=DEFAULT_MAX_BATCH_SIZE,
//					   precisionType precision=TYPE_FASTEST, deviceType device=DEVICE_GPU, bool allowGPUFallback=true );
}

TerrainSegmentation::~TerrainSegmentation()
{
    if (pMask != nullptr) delete[] pMask;// cudaFree(pMask);
    if (pThread != nullptr) {
        ssMain.stop();
        if (pThread->joinable()) pThread->join();
        delete pThread;
    }
}

bool TerrainSegmentation::IsHabitatVisible()
{
    mt.lock();
    bool bReturn = bHabitatVisible;
    mt.unlock();
    return bReturn;
}

float TerrainSegmentation::GetHabitatDirection()
{
    mt.lock();
    float fReturn = fBestDirection;
    mt.unlock();
    return fReturn;
}

void TerrainSegmentation::SetNewImage(CameraFrame& frm)
{
    mt.lock();
    frmRecent.DeepCopy(frm);
    mt.unlock();
    sigNewImage.signal();
}

void TerrainSegmentation::Start()
{
    pThread = new std::thread(&TerrainSegmentation::run, this, sigNewImage, ssMain);
}

void TerrainSegmentation::run(Signal sig, StopSignal ss)
{
    printL("TerrainSegmentation::run");
    CameraFrame frmProcess;
    int iFramesProcessed = 0;

    while (!ss.shouldStop()) {
        sig.wait(1000);
        mt.lock();
        frmProcess = frmRecent; // This doesn't need to be a deep copy?
        mt.unlock();
        if (frmProcess.matrix.empty()) continue;
        
        iFramesProcessed++;
        if (ProcessImage(frmProcess)) {
            logger.LogImage(frmProcess, "segment");
            char sc[128];
            mt.lock();
            sprintf(sc, "Good Habitat Found %2.2f [Image:%d]", fBestDirection, frmProcess.GetId());
            mt.unlock();
            logger.LogString(sc);
        }
    }
    printf("TerrainSegmentation::run Frames Processed: %d\n", iFramesProcessed);
}

void TerrainSegmentation::SendStopSignal()
{
    ssMain.stop();
}

void TerrainSegmentation::SetTargetClass(uint8_t _uTargetClass)
{
    uTargetClass = _uTargetClass;
}

void TerrainSegmentation::SetNumberImageSlices(int _iNumSlices)
{
    iNumSlices = _iNumSlices;
}


void TerrainSegmentation::SetNumberPixelsValid(uint32_t uValid)
{
    uValidNumPixels = uValid;
}

bool TerrainSegmentation::SetNetwork(const char* scNetwork)
{
    if (strcmp("FCN_RESNET18_CITYSCAPES_512x256", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_CITYSCAPES_512x256;
        return true;
    }
    if (strcmp("FCN_RESNET18_CITYSCAPES_1024x512", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_CITYSCAPES_1024x512;
        return true;
    }
    if (strcmp("FCN_RESNET18_CITYSCAPES_2048x1024", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_CITYSCAPES_2048x1024;
        return true;
    }
    if (strcmp("FCN_RESNET18_DEEPSCENE_576x320", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_DEEPSCENE_576x320;
        return true;
    }
    if (strcmp("FCN_RESNET18_DEEPSCENE_864x480", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_DEEPSCENE_864x480;
        return true;
    }
    if (strcmp("FCN_RESNET18_VOC_320x320", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_VOC_320x320;
        return true;
    }
    if (strcmp("FCN_RESNET18_VOC_512x320", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_VOC_512x320;
        return true;
    }
    if (strcmp("FCN_RESNET18_SUNRGB_512x400", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_SUNRGB_512x400;
        return true;
    }
    if (strcmp("FCN_RESNET18_SUNRGB_640x512", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_SUNRGB_640x512;
        return true;
    }
    if (strcmp("FCN_ALEXNET_AERIAL_FPV_720p", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_ALEXNET_AERIAL_FPV_720p;
        return true;
    }
    return false;
}

bool TerrainSegmentation::ProcessImage(CameraFrame& frm)
{
    cv::Mat& imgRGB = frm.matrix;
    int imgWidth = imgRGB.cols;      // width of the image (in pixels)
    int imgHeight = imgRGB.rows;      // height of the image (in pixels)
    // 1080x1920  -- 3 images at 1080x640
    uchar3* u3Img;
    // Going to do iNumSlices horizontal slices of the image. That and it being a 3 channel image is why size is only widthxheight
    if (imgHeight % iNumSlices != 0) printL("ObjectDetection::ProcessImage doesn't divide by 3");
    int iSliceHeight = imgHeight / iNumSlices;
    if (CUDA_FAILED(cudaMalloc(&u3Img, imgWidth * iSliceHeight * 3))) {
        printL("TerrainSegmentation::ProcessImage cudaMalloc fail");
        return false;
    }
    bool bRet = false;
    for (int s = 0; s < iNumSlices; s++) {
        intptr_t imgRGBOffset = s * imgWidth * iSliceHeight;
        cudaMemcpy(u3Img, ((uint8_t*)imgRGB.data) + imgRGBOffset, imgWidth * iSliceHeight * 3, cudaMemcpyHostToDevice);

        if(!pNetwork->Process(u3Img, imgWidth, iSliceHeight)) {
            printf("TerrainSegmentation::ProcessImage fail");
            cudaFree(u3Img);        
            return false;
        }

        // Get mask output resolution for the mask size
        if (pMask == nullptr) {
            uMaskWidth = pNetwork->GetGridWidth();
            uMaskHeight = pNetwork->GetGridHeight();
            pMask = new uint8_t[uMaskWidth * uMaskHeight];
        }
        if(!pNetwork->Mask(pMask, uMaskWidth, uMaskHeight)) {
            printL("TerrainSegmentation::ProcessImage mask fail");
            cudaFree(u3Img);
            return false;
        }
        if (ScanMask(pMask, frm.GetId(), uMaskWidth, uMaskHeight)) {
            // We're starting at the top, which would be the furthest away.
            //  So as soon as we find our target, then just stop scanning
            bRet = true;
            char sc[64];
            sprintf(sc, "ScanMask success: slice %d", s);
            logger.LogString(sc);
            break;
        }
    }
    cudaFree(u3Img);
    
    return bRet;
}

bool TerrainSegmentation::ScanMask(uint8_t* pMask, int iImageId, uint32_t uWidth, uint32_t uHeight)
{
    size_t size = uWidth * uHeight;
    float fAngleAvgSum = 0;
    uint32_t uAngleAvgCount = 0;
    uint8_t pixel;
    // Count up how many valid pixels are in the mask, and average out the direction they come from
    for (size_t u = 0; u < size; u++) {
        pixel = pMask[u];
        if (pixel == uTargetClass) {
            uint32_t x = u / uWidth;
            fAngleAvgSum += (abs(x - (uWidth * .5)) / (uWidth * .5)) * (fImageFOV * .5);
            uAngleAvgCount++;
        }
    }

    // Log the count, and if less than valid pixel count, we return false
    char sc[64];
    sprintf(sc, "ScanMask: count %u[%u %u]", uAngleAvgCount, uWidth, uHeight);
    logger.LogString(sc);
    if (uAngleAvgCount < uValidNumPixels) {
        mt.lock();
        bHabitatVisible = false;
        mt.unlock();
        return false;
    }
    // We've found the desired category, so log the mask
    int maskWidth = uWidth * 4;
    int maskHeight = uHeight * 4;
    float4* imgMask;
    if (!CUDA_FAILED(cudaHostAlloc(&imgMask, maskWidth*maskHeight*16, cudaHostAllocMapped))) {
        pNetwork->Mask(imgMask, maskWidth, maskHeight, segNet::FILTER_POINT);
        logger.LogMask(imgMask, iImageId, maskWidth, maskHeight);
        cudaFreeHost(imgMask);
    }
    // Set direction and that we found something
    mt.lock();
    fBestDirection = fAngleAvgSum / uAngleAvgCount;
    bHabitatVisible = true;
    mt.unlock();
    return true;
}

Navigator::Navigator(FlightController& _control, DataLogger& _logger, PipeReceiverRGB* _pPipeRGB, bool _bStartBatteryMonitor)
: bSendRTL(false)
, bStartBatteryMonitor(_bStartBatteryMonitor)
, iRouteCleanType(0)
, tLastChange(0)
, tMinTimeBetweenChanges(5)
, control(_control)
, logger(_logger)
, camera(_camera)
, pRouteOrig(nullptr)
//, sigMainRGB(_camera.GetSignalRGB())
, pPipeRGB(_pPipeRGB)
, pThread(nullptr)
, detector(logger)
, segmenter(logger)
{

}

Navigator::~Navigator()
{
    if (pThread != nullptr) {
        ssMain.stop();
        if (pThread->joinable()) pThread->join();
        delete pThread;
    }
    if (pRouteOrig != nullptr) {
        delete pRouteOrig;
    }
    if (pPipeRGB != nullptr) {
        delete pPipeRGB;
    }
}
    
void Navigator::Start()
{
    pThread = new std::thread(&Navigator::runNavigator, this, std::ref(*pPipeRGB), ssMain);
}

void Navigator::Join()
{
    if (pThread != nullptr && pThread->joinable()) {
        pThread->join();
    }
}

void Navigator::SignalStop()
{
    if (pThread != nullptr) {
        ssMain.stop();
    }
}

void Navigator::SetNumberImageSlices(int iNumSlices)
{
    detector.SetNumberImageSlices(iNumSlices);
    segmenter.SetNumberImageSlices(iNumSlices);
}

void Navigator::SetTimeBetweenChanges(int iTime)
{
    tMinTimeBetweenChanges = time_t(iTime);
}

void Navigator::SetValidPixels(uint32_t uValidPixels)
{
    segmenter.SetNumberPixelsValid(uValidPixels);
}

void Navigator::SetRouteClean(int iClean)
{
    iRouteCleanType = iClean;
}

void Navigator::runNavigator(PipeReceiverRGB pipeRGB, StopSignal ss)
{
    printL("Navigator::runNavigator");
    pthread_setname_np(pthread_self(), "Navigator::runNavigator");

    bool bGridSearching = true;
    float fMargin = 5;
    float fCurrentDirection = 0;

    StopSignal ssBattery;
    std::thread* pThrBattery = nullptr;
    if (bStartBatteryMonitor) pThrBattery = new std::thread(&Navigator::runBattery, this, ssBattery, ssMain);
    
    detector.Start();
    segmenter.Start();

    Route* pRouteCurrent = nullptr;
    pRouteOrig = control.GetRoute();
    if (pRouteOrig) {
        logger.LogString(pRouteOrig->toString().c_str());
        pRouteCurrent = new Route(*pRouteOrig);
    } else {
        printL(" Failed to get route.");
        logger.LogString("Failed to get route.");
    }

    while (!ss.shouldStop()) {
        //sigRGB.wait(1000);//todo???timeout?
        std::shared_ptr<cv::Mat> pNewImage;
        pipeRGB.RetrieveData(pNewImage);
        // Get new image and process it
        if (!pipeRGB.RetrieveData(pNewImage)) {
            printL("runNavigator frame was not valid");
            continue;
        }
        CameraFrame frameNew(*pNewImage);
        detector.SetNewImage(frameNew);
        segmenter.SetNewImage(frameNew);

        if (bGridSearching) { // Habitat not visible, we're currently running search pattern
            // Good terrain visible?
            if (segmenter.IsHabitatVisible()) {
                char sc[64];
                // If enough time since last command, then we'll change
                if (time(nullptr) - tLastChange > tMinTimeBetweenChanges) {
                    float fDir = segmenter.GetHabitatDirection();
                    // Verify direction is valid before we switch to guided
                    int iDistanceM = control.CheckDirectionValid(fDir, tMinTimeBetweenChanges);
                    if (iDistanceM > 0) {
                        control.SetDestination(fDir, iDistanceM);
                        fCurrentDirection = fDir;
                        sprintf(sc, "Grid Search -> Good Habitat %1.3f", segmenter.GetHabitatDirection());
                        bGridSearching = false;
                        tLastChange = time(nullptr);
                    } else {
                        sprintf(sc, "Grid Search X Good Habitat %1.3f (Invalid Direction)", segmenter.GetHabitatDirection());
                    }
                } else {
                    sprintf(sc, "[delay]Grid Search -> Good Habitat %1.3f", segmenter.GetHabitatDirection());
                }
                logger.LogString(sc);
            }
        } else { // Habitat already visible
            // Good terrain still visible?
	        if (segmenter.IsHabitatVisible()) {
                char sc[64];
                // If enough time since last command, then we can change
                if (time(nullptr) - tLastChange > tMinTimeBetweenChanges) {
                    float fDir = segmenter.GetHabitatDirection();
                    // Verify that the change won't take us in an invalid direction
                    int iDistanceM = control.CheckDirectionValid(fDir, tMinTimeBetweenChanges);
                    if (iDistanceM > 0) {
                        control.SetDestination(fDir, iDistanceM);
                        sprintf(sc, "Good Habitat Change -> %1.3f", segmenter.GetHabitatDirection());
                        tLastChange = time(nullptr);
                    } else {
                        // If it would be an invalid direction, then go back to grid search
                        pRouteCurrent = RestoreRoute(pRouteCurrent);
                        sprintf(sc, "Good Habitat (Invalid Direction) -> %1.3f", segmenter.GetHabitatDirection());
                        bGridSearching = true;
                        tLastChange = time(nullptr);
                    }
                } else {
                    sprintf(sc, "[delay]Good Habitat Change -> %1.3f", segmenter.GetHabitatDirection());
                }
                logger.LogString(sc);
            } else { // No good terrain is visible right now
                // So if there's been enough time we'll switch back to grid
                if (time(nullptr) - tLastChange > tMinTimeBetweenChanges) {
                    pRouteCurrent = RestoreRoute(pRouteCurrent);
                    logger.LogString("Good Habitat -> Grid Search");
                    bGridSearching = true;
                    tLastChange = time(nullptr);
                } else {
                    logger.LogString("[delay]Good Habitat -> Grid Search");
                }
            }
        }
    }

    if (pRouteOrig) {
        delete pRouteOrig;
        pRouteOrig = nullptr;
    }
    if (pRouteCurrent) {
        delete pRouteCurrent;
        pRouteCurrent = nullptr;
    }
    printL("Navigator::runNavigator loop done, stopping segmenter and detector");
    // Handle all the cleanup
    detector.SendStopSignal();
    segmenter.SendStopSignal();
    
    if (bStartBatteryMonitor) {
        ssBattery.stop();
        if (pThrBattery->joinable()) pThrBattery->join();
        delete pThrBattery;
        if (bSendRTL) control.SetRTL();
    }
}

Route* Navigator::RestoreRoute(Route* pRoute)
{
    if (pRoute == nullptr) return nullptr;

    if (iRouteCleanType == 0) return pRoute;

    // Get any visited points
    CoordNode* pVisited = control.GetVisitedPoints();
    // Clean the route (get next waypoint so we can skip every one we've visited)
    int16_t iWaypoint = 1;
    if (control.GetNextWaypoint(iWaypoint)) {
        bool bVisit = pVisited != nullptr;
        char sc[512];
        snprintf(sc, 512, "Restoring route; VisitedPoints: %s ; NextWaypoint: %d\n", bVisit ? "YES" : "NO", iWaypoint);
        logger.LogString(sc);
    } else {
        char sc[512];
        snprintf(sc, 512, "Restoring route; VisitedPoints: %s ; NextWaypoint: NO\n", pVisited != nullptr ? "YES" : "NO");
        logger.LogString(sc);
    }

    pRoute = Route::CleanRoute(pRoute, pVisited, iWaypoint, iRouteCleanType == 2);
    delete pVisited;
    if (pRoute == nullptr) {
        printL(" Route failure"); // This shouldn't happen
        return nullptr;
    }

    // Send message with new route to the flight controller
    control.SetRoute(*pRoute, true);
    logger.LogString(pRoute->toString().c_str());
    
    return pRoute;
}

void Navigator::runBattery(StopSignal ss, StopSignal ssMainThread)
{
    printL("Navigator::runBattery");

    // Loop over uCheckBattery, on each overflow we'll check the battery. This way we don't have to check very often,
    //  but don't wait too long while tryin to stop this thread.
    uint8_t uCheckBattery = 0;
    while (!ss.shouldStop()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (uCheckBattery++ != 0) continue;

        int iBattery = control.GetBattery();
        if (iBattery == -1) continue; // -1 we didn't get a response so just continue

        // For now, simple method...if battery is < 50% then we will stop the nav threads and RTL.
        if (iBattery < 50) {
            bSendRTL = true;
            ssMainThread.stop();
            break;
        }
    }
}
